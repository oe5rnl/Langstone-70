#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Lang Rx
# Generated: Mon May 25 12:31:53 2020
##################################################


from gnuradio import analog
from gnuradio import audio
from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio import iio
from gnuradio.eng_option import eng_option
from gnuradio.fft import logpwrfft
from gnuradio.filter import firdes
from grc_gnuradio import blks2 as grc_blks2
from optparse import OptionParser
 
import os
import errno

class Lang_RX(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "Lang Rx")

        ##################################################
        # Variables
        ##################################################
        self.USB = USB = True
        self.SQL = SQL = 50
        self.RxOffset = RxOffset = 0
        self.NCW = NCW = False
        self.Mute = Mute = False
        self.FM = FM = False
        self.FFTEn = FFTEn = 0
        self.CW = CW = False
        self.AFGain = AFGain = 20

        ##################################################
        # Blocks
        ##################################################
        self.pluto_source_0 = iio.pluto_source('ip:pluto.local', 1000000000, 576000, 2000000, 0x800, True, True, True, "slow_attack", 64.0, '', True)
        self.logpwrfft_x_0 = logpwrfft.logpwrfft_c(
        	sample_rate=576000,
        	fft_size=512,
        	ref_scale=2,
        	frame_rate=15,
        	avg_alpha=0.9,
        	average=True,
        )
        self.freq_xlating_fir_filter_xxx_0 = filter.freq_xlating_fir_filter_ccc(12, (firdes.low_pass(1,576000,20000,6000)), RxOffset, 576000)
        self.blocks_null_sink_0 = blocks.null_sink(gr.sizeof_float*512)
        self.blocks_multiply_const_vxx_2_0 = blocks.multiply_const_vff((int(FM), ))
        self.blocks_multiply_const_vxx_2 = blocks.multiply_const_vff((not FM, ))
        self.blocks_multiply_const_vxx_1 = blocks.multiply_const_vff(((AFGain/100.0) *  (not Mute), ))
        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_float*512, '/tmp/langstonefft', False)
        self.blocks_file_sink_0.set_unbuffered(False)
        self.blocks_complex_to_real_0 = blocks.complex_to_real(1)
        self.blocks_add_xx_1 = blocks.add_vff(1)
        self.blks2_selector_0 = grc_blks2.selector(
        	item_size=gr.sizeof_float*512,
        	num_inputs=1,
        	num_outputs=2,
        	input_index=0,
        	output_index=FFTEn,
        )
        self.band_pass_filter_0 = filter.fir_filter_ccc(1, firdes.complex_band_pass(
        	1, 48000, ((-3000+USB*3300+NCW*CW*250)*(1-FM)) + (-7500 * FM), ((-300+USB*3300-NCW*CW*1950)* (1-FM)) + (7500 * FM), 100, firdes.WIN_HAMMING, 6.76))
        self.audio_sink_0 = audio.sink(48000, "hw:CARD=Device,DEV=0", False)
        self.analog_pwr_squelch_xx_0 = analog.pwr_squelch_cc(SQL-100, 0.001, 0, False)
        self.analog_nbfm_rx_0 = analog.nbfm_rx(
        	audio_rate=48000,
        	quad_rate=48000,
        	tau=75e-6,
        	max_dev=5e3,
          )
        self.analog_agc2_xx_0 = analog.agc2_ff(1e-1, 1e-1, 0.1, 1)
        self.analog_agc2_xx_0.set_max_gain(1000)



        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_agc2_xx_0, 0), (self.blocks_add_xx_1, 0))
        self.connect((self.analog_nbfm_rx_0, 0), (self.blocks_multiply_const_vxx_2_0, 0))
        self.connect((self.analog_pwr_squelch_xx_0, 0), (self.analog_nbfm_rx_0, 0))
        self.connect((self.band_pass_filter_0, 0), (self.analog_pwr_squelch_xx_0, 0))
        self.connect((self.band_pass_filter_0, 0), (self.blocks_complex_to_real_0, 0))
        self.connect((self.blks2_selector_0, 1), (self.blocks_file_sink_0, 0))
        self.connect((self.blks2_selector_0, 0), (self.blocks_null_sink_0, 0))
        self.connect((self.blocks_add_xx_1, 0), (self.blocks_multiply_const_vxx_1, 0))
        self.connect((self.blocks_complex_to_real_0, 0), (self.blocks_multiply_const_vxx_2, 0))
        self.connect((self.blocks_multiply_const_vxx_1, 0), (self.audio_sink_0, 0))
        self.connect((self.blocks_multiply_const_vxx_2, 0), (self.analog_agc2_xx_0, 0))
        self.connect((self.blocks_multiply_const_vxx_2_0, 0), (self.blocks_add_xx_1, 1))
        self.connect((self.freq_xlating_fir_filter_xxx_0, 0), (self.band_pass_filter_0, 0))
        self.connect((self.logpwrfft_x_0, 0), (self.blks2_selector_0, 0))
        self.connect((self.pluto_source_0, 0), (self.freq_xlating_fir_filter_xxx_0, 0))
        self.connect((self.pluto_source_0, 0), (self.logpwrfft_x_0, 0))

    def get_USB(self):
        return self.USB

    def set_USB(self, USB):
        self.USB = USB
        self.band_pass_filter_0.set_taps(firdes.complex_band_pass(1, 48000, ((-3000+self.USB*3300+self.NCW*self.CW*250)*(1-self.FM)) + (-7500 * self.FM), ((-300+self.USB*3300-self.NCW*self.CW*1950)* (1-self.FM)) + (7500 * self.FM), 100, firdes.WIN_HAMMING, 6.76))

    def get_SQL(self):
        return self.SQL

    def set_SQL(self, SQL):
        self.SQL = SQL
        self.analog_pwr_squelch_xx_0.set_threshold(self.SQL-100)

    def get_RxOffset(self):
        return self.RxOffset

    def set_RxOffset(self, RxOffset):
        self.RxOffset = RxOffset
        self.freq_xlating_fir_filter_xxx_0.set_center_freq(self.RxOffset)

    def get_NCW(self):
        return self.NCW

    def set_NCW(self, NCW):
        self.NCW = NCW
        self.band_pass_filter_0.set_taps(firdes.complex_band_pass(1, 48000, ((-3000+self.USB*3300+self.NCW*self.CW*250)*(1-self.FM)) + (-7500 * self.FM), ((-300+self.USB*3300-self.NCW*self.CW*1950)* (1-self.FM)) + (7500 * self.FM), 100, firdes.WIN_HAMMING, 6.76))

    def get_Mute(self):
        return self.Mute

    def set_Mute(self, Mute):
        self.Mute = Mute
        self.blocks_multiply_const_vxx_1.set_k(((self.AFGain/100.0) *  (not self.Mute), ))

    def get_FM(self):
        return self.FM

    def set_FM(self, FM):
        self.FM = FM
        self.blocks_multiply_const_vxx_2_0.set_k((int(self.FM), ))
        self.blocks_multiply_const_vxx_2.set_k((not self.FM, ))
        self.band_pass_filter_0.set_taps(firdes.complex_band_pass(1, 48000, ((-3000+self.USB*3300+self.NCW*self.CW*250)*(1-self.FM)) + (-7500 * self.FM), ((-300+self.USB*3300-self.NCW*self.CW*1950)* (1-self.FM)) + (7500 * self.FM), 100, firdes.WIN_HAMMING, 6.76))

    def get_FFTEn(self):
        return self.FFTEn

    def set_FFTEn(self, FFTEn):
        self.FFTEn = FFTEn
        self.blks2_selector_0.set_output_index(int(self.FFTEn))

    def get_CW(self):
        return self.CW

    def set_CW(self, CW):
        self.CW = CW
        self.band_pass_filter_0.set_taps(firdes.complex_band_pass(1, 48000, ((-3000+self.USB*3300+self.NCW*self.CW*250)*(1-self.FM)) + (-7500 * self.FM), ((-300+self.USB*3300-self.NCW*self.CW*1950)* (1-self.FM)) + (7500 * self.FM), 100, firdes.WIN_HAMMING, 6.76))

    def get_AFGain(self):
        return self.AFGain

    def set_AFGain(self, AFGain):
        self.AFGain = AFGain
        self.blocks_multiply_const_vxx_1.set_k(((self.AFGain/100.0) *  (not self.Mute), ))

def docommands(tb):
  try:
    os.mkfifo("/tmp/langstoneRx")
  except OSError as oe:
    if oe.errno != errno.EEXIST:
      raise    
  ex=False
  lastbase=0
  while not ex:
    fifoin=open("/tmp/langstoneRx",'r')
    while True:
       try:
        with fifoin as filein:
         for line in filein:
           line=line.strip()
           if line=='Q':
              ex=True        
           if line=='U':
              tb.set_USB(True)
              tb.set_FM(False)
              tb.set_CW(False)              
           if line=='L':
              tb.set_USB(False)
              tb.set_FM(False)
              tb.set_CW(False) 
           if line=='F':
              tb.set_FM(True)             
           if line=='C':
              tb.set_CW(True)
              tb.set_FM(False)
              tb.set_USB(True) 
           if line=='N':
              tb.set_NCW(True)
           if line=='W':
              tb.set_NCW(False) 
           if line=='P':
              tb.set_FFTEn(1)
           if line=='p':
              tb.set_FFTEn(0)
           if line=='M':
              tb.set_Mute(1)
           if line=='m':
              tb.set_Mute(0)
           if line[0]=='O':
              value=int(line[1:])
              tb.set_RxOffset(value)  
           if line[0]=='V':
              value=int(line[1:])
              tb.set_AFGain(value)
           if line[0]=='Z':
              value=int(line[1:])
              tb.set_SQL(value) 
           if line=='H':
              tb.lock()
           if line=='h':
              tb.unlock()                            
       except:
         break



def main(top_block_cls=Lang_RX, options=None):

    tb = top_block_cls()
    tb.start()
    docommands(tb)
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
