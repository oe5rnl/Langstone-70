
##################################################
# Piped Commands Tx Control Thread for Hayling Transceiver
# Author: G4EML
# Needs to be manually added into Gnu Radio Flowgraph
##################################################

####################################################
#Add these imports to the top
#####################################################
 
import os
import errno

#######################################################
# Manually add just before the Main () Function
# to provide support for Piped commands
#######################################################
def docommands(tb):
  try:
    os.mkfifo("/tmp/langstoneTx")
  except OSError as oe:
    if oe.errno != errno.EEXIST:
      raise    
  ex=False
  lastbase=0
  while not ex:
    fifoin=open("/tmp/langstoneTx",'r')
    while True:
       try:
        with fifoin as filein:
         for line in filein:
           line=line.strip()
           if line=='Q':
              ex=True        
           if line=='R':
              tb.set_PTT(False) 
           if line=='T':
              tb.set_PTT(True)
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
           if line=='K':
              tb.set_KEY(True) 
           if line=='k':
              tb.set_KEY(False)    
           if line[0]=='G':
              value=int(line[1:])
              tb.set_MicGain(value) 
           if line[0]=='g':
              value=int(line[1:])
              tb.set_FMMIC(value)
           if line=='H':
              tb.lock()
           if line=='h':
              tb.unlock()                      
       except:
         break

########################################################


#########################################################
#Replace the Main() function with this
########################################################
    tb = top_block_cls()
    tb.start()
    docommands(tb)
    tb.stop()
    tb.wait()
#########################################################
