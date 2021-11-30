import asyncio
import datetime
import socket
import struct
from copy import deepcopy
from functools import partial

import numpy as np

import crcmod
crc16 = crcmod.predefined.mkPredefinedCrcFun('modbus')

import astropy.units as u
from astropy.coordinates import Angle

from bokeh.events import ButtonClick
from bokeh.io import curdoc
from bokeh.layouts import column, row
from bokeh.models import Button, ColumnDataSource, Div, HoverTool, TextInput
from bokeh.palettes import Colorblind8
from bokeh.plotting import figure

DEFAULT_REMOTE_IP = "192.168.2.102"
DEFAULT_SELF_IP = "192.168.2.101"

CMD_PORT = 50501
TM_PORT = 60501


command_packet = bytearray()
command_sequence_number = -1
acknowledged_command_sequence_number = -1


telemetry_info = np.zeros((256, 256))
telemetry_info_lock = asyncio.Lock()

data_rates = ColumnDataSource(data=dict(date=[], total=[], interface=[], imager_hk=[], imager_event=[], gps=[]))


statistics = ColumnDataSource(data=dict(date=[],
                                        events_0=[], remain_0=[], bad_0=[],
                                        events_1=[], remain_1=[], bad_1=[],
                                        events_2=[], remain_2=[], bad_2=[],
                                        events_3=[], remain_3=[], bad_3=[],
                                        events_4=[], remain_4=[], bad_4=[],
                                        events_5=[], remain_5=[], bad_5=[],
                                        events_6=[], remain_6=[], bad_6=[],
                                        events_7=[], remain_7=[], bad_7=[]
                                       ))


gps_info = {'hour': 0,
            'minute': 0,
            'second': 0,
            'latitude': Angle(0*u.deg),
            'longitude': Angle(0*u.deg),
            'altitude': 0*u.m,
            'quality': "",
            'num_sat': 0,
            'hdop': 0,
            'geoidal': 0*u.m,
            'gondola': 0,
           }
gps_info_lock = asyncio.Lock()
quality_strings = ['invalid fix', 'GPS fix', 'SBAS-corrected fix', '', '', '','', 'cached']


pps_info = {'day_offset': 0,
            'hour': 0,
            'minute': 0,
            'second': 0,
            'clock_difference': 0*u.s,
            'gondola': 0,
            'datetime' : datetime.datetime(2000, 1, 1)
           }
pps_info_lock = asyncio.Lock()


timing = ColumnDataSource(data=dict(date=[], gps_to_pps=[], pps_to_sbc=[]))



"""
Telemetry parsing
"""

async def parse_statistics(data):
    async with pps_info_lock:
        global pps_info
        date = deepcopy(pps_info['datetime'])

    new_statistics = dict(date=[date])
    for i in range(8):
        new_statistics[f'events_{i}'] = [struct.unpack('H', data[6*i + 16: 6*i + 18])[0]]
        new_statistics[f'remain_{i}'] = [struct.unpack('H', data[6*i + 18: 6*i + 20])[0]]
        bad_bytes = struct.unpack('H', data[6*i + 20: 6*i + 22])[0]
        new_statistics[f'bad_{i}'] = [bad_bytes]
        if bad_bytes > 0:
            print(f"Gondola time: {data[10:16].hex()}, Imager: {i}, Number of bad bytes: {bad_bytes}")

    doc.add_next_tick_callback(partial(statistics.stream, new_statistics))


async def parse_gps_position(data):
    async with gps_info_lock:
        global gps_info
        gps_info['hour'] = data[16]
        gps_info['minute'] = data[17]
        gps_info['second'] = data[18]
        gps_info['latitude'] = Angle(struct.unpack('d', data[20:28])[0]*u.deg)
        gps_info['longitude'] = Angle(struct.unpack('d', data[28:36])[0]*u.deg)
        gps_info['altitude'] = struct.unpack('H', data[42:44])[0]*u.m
        gps_info['quality'] = quality_strings[data[36]]
        gps_info['num_sat'] = data[37]
        gps_info['hdop'] = struct.unpack('f', data[38:42])[0]
        gps_info['geoidal'] = struct.unpack('h', data[44:46])[0]*u.m
        gps_info['gondola'] = struct.unpack('q', data[10:16] + b'00')[0] / 1e7

        if data[36] != 1:
            #print(gps_info)
            pass

    doc.add_next_tick_callback(update_gps_info_div)


async def parse_pps(data):
    async with pps_info_lock:
        global pps_info
        pps_info['day_offset'] = data[16]
        pps_info['hour'] = data[17]
        pps_info['minute'] = data[18]
        pps_info['second'] = data[19]
        pps_info['clock_difference'] = struct.unpack('l', data[20:24])[0]*u.us
        pps_info['second_offset'] = struct.unpack('L', data[24:28])[0]*u.s
        pps_info['gondola'] = struct.unpack('q', data[10:16] + b'00')[0] / 1e7

        pps_info['datetime'] = datetime.datetime(2000, 1, 1 + pps_info['day_offset'],
                                                 pps_info['hour'], pps_info['minute'], pps_info['second'])
        if pps_info['day_offset'] == 0 and pps_info['hour'] == 0 and pps_info['minute'] == 0 and pps_info['second'] == 0:
            pps_info['datetime'] += datetime.timedelta(0, pps_info['second_offset'].to_value('s'))

        date = deepcopy(pps_info['datetime'])

        async with gps_info_lock:
            global gps_info
            if gps_info['gondola'] > 0:  # if at least one GPS position packet has been received
                #if pps_info['hour'] == 0 and pps_info['minute'] == 0:
                #    print(pps_info)
                #    print(gps_info)
                #if pps_info['second_offset'] != 1*u.s:
                #    print(pps_info)
                #    print(gps_info)
                gps_to_pps = pps_info['gondola'] - gps_info['gondola']
                new_timing = dict(date=[date],
                                  gps_to_pps=[gps_to_pps],
                                  pps_to_sbc=[pps_info['clock_difference'].to_value('s')])
                doc.add_next_tick_callback(partial(timing.stream, new_timing))

        async with telemetry_info_lock:
            global telemetry_info
            new_data_rates = dict(date=[date],
                                  total=[np.sum(telemetry_info)],
                                  interface=[np.sum(telemetry_info[0xa0, :])],
                                  imager_hk=[np.sum(telemetry_info[0xc0:0xc8, 0x09:0x10])],
                                  imager_event=[np.sum(telemetry_info[0xc0:0xc8, 0xc0])],
                                  gps=[np.sum(telemetry_info[0x60, 0x60:0x63])]
                                 )
            telemetry_info = np.zeros((256, 256))
            doc.add_next_tick_callback(partial(data_rates.stream, new_data_rates))

    doc.add_next_tick_callback(update_pps_info_div)


class TelemetryProtocol:
    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        loop = asyncio.get_running_loop()

        tm = bytearray(data)
        tm[2:4] = [0, 0]
        hb, lb = divmod(crc16(tm), 256)
        if data[2:4] != bytearray([lb, hb]):
            print("Bad checksum")
            return

        sysid = tm[4]
        tmtype = tm[5]

        if tmtype == 0x01:
            global acknowledged_command_sequence_number
            acknowledged_command_sequence_number = int.from_bytes(tm[8:10], "little")

            doc.add_next_tick_callback(update_command_info_div)
        elif (sysid, tmtype) == (0x60, 0x60):
            loop.create_task(parse_pps(tm))
        elif (sysid, tmtype) == (0x60, 0x61):
            loop.create_task(parse_gps_position(tm))
        elif (sysid, tmtype) == (0x60, 0x62):
            # TODO
            pass
        elif (sysid, tmtype) == (0xa0, 0x0c):
            loop.create_task(parse_statistics(tm))
        elif sysid & 0xc0 == 0xc0:
            # TODO
            pass
        else:
            print(f"Unhandled telemetry packet (0x{sysid:02x}/0x{tmtype:02x})")

        loop.create_task(update_telemetry_info(sysid, tmtype, len(tm)))


async def update_telemetry_info(sysid, tmtype, size):
    async with telemetry_info_lock:
        global telemetry_info
        telemetry_info[sysid, tmtype] += size


"""
Commanding
"""

def update_command_info_div():
    command_info_div.text = (f"Command #{command_sequence_number}: {command_packet.hex()}<br>"
                             f"Acknowledgement: #{acknowledged_command_sequence_number}")


def send_command(sysid, cmdtype, payload=[]):
    global command_packet, command_sequence_number
    command_sequence_number += 1

    payload = bytearray(payload)
    command_packet = (bytearray.fromhex('90eb0000') +
                      bytearray([sysid, cmdtype, command_sequence_number, len(payload)]) +
                      payload)

    hb, lb = divmod(crc16(command_packet), 256)
    command_packet[2:4] = [lb, hb]

    print(f"Sending {command_packet.hex()} to {remote_ip_text.value}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(command_packet, (remote_ip_text.value, CMD_PORT))

    doc.add_next_tick_callback(update_command_info_div)


def route_telemetry_command():
    send_command(0xa0, 0xa1, map(int, self_ip_text.value.split('.')))


def send_raw_command():
    if len(raw_command_text.value) >= 4:
        raw_command = bytearray.fromhex(raw_command_text.value)
        send_command(raw_command[0], raw_command[1], raw_command[2:])


"""
GPS
"""

def update_gps_info_div():
    gps_info_div.text = (f"GPS time: {gps_info['hour']:02}:{gps_info['minute']:02}:{gps_info['second']:02}<br>"
                         f"Lat: {gps_info['latitude'].to_string()}<br>"
                         f"Lon: {gps_info['longitude'].to_string()}<br>"
                         f"Alt: {gps_info['altitude']}<br>"
                         f"# of satellites: {gps_info['num_sat']}<br>"
                         f"Quality: {gps_info['quality']}<br>"
                         f"HDOP: {gps_info['hdop']}<br>"
                         f"Geoidal sep: {gps_info['geoidal']}<br>"
                         f"Gondola: {gps_info['gondola']}<hr>")


"""
PPS
"""

def update_pps_info_div():
    pps_info_div.text = (f"GPS time at PPS: {pps_info['hour']:02}:{pps_info['minute']:02}:{pps_info['second']:02} + {pps_info['day_offset']}d<br>"
                         f"Clock diff: {pps_info['clock_difference']}<br>"
                         f"Applied offset: {pps_info['second_offset']}<br>"
                         f"Gondola: {pps_info['gondola']}")


"""
Interface layout
"""

def create_hover_tool(list_of_keys):
    tooltips = [('date', '@date{%F %T}')] + [(s, '@'+s) for s in list_of_keys]
    return HoverTool(tooltips=tooltips, formatters={'@date': 'datetime'}, mode='mouse')


remote_ip_text = TextInput(title="Remote IP", value=DEFAULT_REMOTE_IP)
self_ip_text = TextInput(title="Self IP", value=DEFAULT_SELF_IP)

route_button = Button(label="Route telemetry")
route_button.on_event(ButtonClick, route_telemetry_command)

raw_command_text = TextInput(title="Raw command (hex)")

send_raw_command_button = Button(label="Send raw command")
send_raw_command_button.on_event(ButtonClick, send_raw_command)

command_info_div = Div(text="")

command_block = column(remote_ip_text,
                       self_ip_text,
                       route_button,
                       raw_command_text,
                       send_raw_command_button,
                       command_info_div)

gps_info_div = Div(text="")
pps_info_div = Div(text="")

gps_block = column(gps_info_div, pps_info_div)


tools = "box_zoom,crosshair,pan,reset,save,wheel_zoom"


data_rates_plot = figure(plot_width=400, plot_height=300, x_axis_type='datetime', tools=tools,
                         y_axis_label='bytes/s', title='Telemetry data rates')
data_rates_plot.add_tools(create_hover_tool(['total', 'interface', 'gps', 'imager_hk', 'imager_event']))
data_rates_plot.line('date', 'total', source=data_rates, line_color=Colorblind8[7], legend_label='Total')
data_rates_plot.line('date', 'interface', source=data_rates, line_color=Colorblind8[0], legend_label='Interface')
data_rates_plot.line('date', 'gps', source=data_rates, line_color=Colorblind8[1], legend_label='GPS & PPS')
data_rates_plot.line('date', 'imager_hk', source=data_rates, line_color=Colorblind8[2], legend_label='Imager H/K')
data_rates_plot.line('date', 'imager_event', source=data_rates, line_color=Colorblind8[3], legend_label='Imager Events')

data_rates_plot.legend.location = "top_left"
data_rates_plot.legend.click_policy = "hide"


event_rates_plot = figure(plot_width=900, plot_height=300, x_axis_type='datetime', tools=tools,
                         y_axis_label='events/s', title='Imager event rates')
event_rates_plot.add_tools(create_hover_tool([f'events_{i}' for i in range(8)]))
for i in range(8):
    event_rates_plot.line('date', f'events_{i}', source=statistics, line_color=Colorblind8[i], legend_label=f'Imager {i}')

event_rates_plot.legend.location = "top_left"
event_rates_plot.legend.click_policy = "hide"


gps_to_pps_plot = figure(plot_width=900, plot_height=200, x_axis_type='datetime', tools=tools,
                         y_axis_label='seconds', title='Gondola time of PPS signal minus preceding GPS position packet')
gps_to_pps_plot.add_tools(create_hover_tool(['gps_to_pps']))
gps_to_pps_plot.line('date', 'gps_to_pps', source=timing, line_color=Colorblind8[0])


pps_to_sbc_plot = figure(plot_width=900, plot_height=200, x_axis_type='datetime', tools=tools,
                         y_axis_label='seconds', title='System clock minus GPS clock')
pps_to_sbc_plot.add_tools(create_hover_tool(['pps_to_sbc']))
pps_to_sbc_plot.line('date', 'pps_to_sbc', source=timing, line_color=Colorblind8[1])


doc = curdoc()
doc.add_root(column(row(command_block, data_rates_plot, gps_block), event_rates_plot, gps_to_pps_plot, pps_to_sbc_plot))
doc.title = "Middleman GSE"


"""
Execution
"""

doc.add_next_tick_callback(update_command_info_div)
doc.add_next_tick_callback(update_gps_info_div)

async def setup_udp_listening():
    loop = asyncio.get_running_loop()
    transport, protocol = await loop.create_datagram_endpoint(lambda: TelemetryProtocol(),
                                                              local_addr=("0.0.0.0", TM_PORT))


loop = asyncio.get_running_loop()
loop.create_task(setup_udp_listening())
