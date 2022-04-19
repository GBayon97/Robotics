import RPi.GPIO as GP

cs=24
clk=23
mosi=19
miso=21
GP.setwarnings(False)
GP.setmode(GP.BOARD)
GP.setup(mosi,GP.OUT)
GP.setup(clk,GP.OUT)
GP.setup(cs,GP.OUT)
GP.setup(miso,GP.IN)

GP.output(cs,GP.HIGH)
GP.output(clk,GP.LOW)
GP.output(mosi,GP.LOW)

def volts(data):
    return (data*3.3)/float(4096)

def read(channel):
    adc=0
    bits=0b11000000
    bits|=((channel-1)<<3)
    GP.output(cs,GP.LOW)
    
    for i in range(7,2,-1):
        valor=(bits&1<<i)
        GP.output(mosi,valor)
        GP.output(clk,GP.HIGH)
        GP.output(clk,GP.LOW)
        
    GP.output(clk,GP.HIGH)
    GP.output(clk,GP.LOW)
    GP.output(clk,GP.HIGH)
    GP.output(clk,GP.LOW)

    lst=[]
    aux=[]
    for i in range(11,-1,-1):
        v=GP.input(miso)
        adc+=(v<<i)
        lst.append(v)
        GP.output(clk,GP.HIGH)
        GP.output(clk,GP.LOW)

    GP.output(cs,GP.HIGH)
    return adc

class LDR:
    def __init__(self,channel=1):
        self.channel=channel
        
    def value(self):
        level=read(self.channel)
        voltage=volts(level)
        return voltage
class IR:
    def __init__(self,channel=2):
        self.channel=channel
        
    def value(self):
        level=read(self.channel)
        dist=135.422261*(level**-0.990808) #devuelve metros
        return round(dist*100,4)
