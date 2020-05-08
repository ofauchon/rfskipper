
class Signal {
    
    
    position = 0;                    // Current position of decoder
    timePosition = 0;                    // Current position of decoder
    
    decoded = "";               // Decoded string
    shortPulseDuration = 150;    // Duration of short pulse
    trace = [];                     // Array of pulses
    
    errorMarginPct = 0.2
    
    constructor() {
    }
    
    reset() {
        this.position = 0;
        this.timePosition=0; 
    }
    
    setPosition(pPosition) {
        this.position = pPosition;
    }
    
    getPosition() {
        return this.position;
    }
    
    setTimePosition(pTimePosition) {
        this.timePosition = pTimePosition;
    }
    
    getTimePosition() {
        return this.timePosition;
    }
    
    getDecoded() {
        return this.decoded;
    }
    
    setShortPulseDuration(pMicrosec) {
        this.shortPulseDuration = pMicrosec;
    }
    
    setTrace(pTrace) {
        this.trace = pTrace;
    }
    
    getTrace() {
        return this.trace;
    }
    
    pulseToTime(pPulse){
        var ret=0; 
        for (var k=0; k<pPulse; k++){
            ret += this.trace[k]
        }
        return ret;
    }

    debug(pLog){
        if (typeof window === 'undefined'){
            console.log(pLog)
        } else {
            //console1(pLog)
            console.log(pLog)
        }
    }

    // Read Bytes from trace
    readBytes(pNumBytes) {
        var ret = [];
        var bitcount = 0;
        var u8val = 0;
        var state = 0;
        var pShortPulseDuration = this.shortPulseDuration * (1-this.errorMarginPct)
        while (ret.length < pNumBytes && this.position < this.trace.length) {
            state = ((this.position % 2) == 0) ? "1" : "0"
            var tCnt = Math.floor(this.trace[this.position] / pShortPulseDuration)
            
            for (var k = 0; k < tCnt; k++) {
                u8val = u8val << 1;
                if (state == "1") {
                    u8val |= 1
                }
                bitcount++;
                if (bitcount == 8) {
                    bitcount = 0;
                    ret.push(u8val)
                    //this.debug("Push:" + u8val)
                    u8val = 0
                }
            }
            
            this.timePosition += this.trace[this.position]
            this.position++
            //debug("Position:" + this.position)
        }
        return ret;
    }
    
    /*
     * Converts byte array to bit array
     */ 
    static bytesToBits(pBytes){
        var ret = new Array; 
        for (var i=0; i< pBytes.length; i++ ){ // Walk the bytes array
            var byt=pBytes[i]  // Current byte
            for (var j=7; j>=0; j-- ){  // down to bits (from left to right)
                var bit = ( byt & (1 <<  j) )  >0
               // console.log("byte #" + i + " bit #" + j + " value: " + bit)
                ret.push(bit);
            } 
        }
        return ret
    }

// 0x56 0x59 => 86 89 => ‭01010110‬‬ ‭01011001‬
// Manchester             1 1 1 0  1 1 0 1 => 0xED (237)
    decodeManchester2(pBits){
        // copy bits to array
        //var bits = Signal.bytesToBits(pBytes)

        var res = []
        var r=0 
        var cnt =0;
        while (pBits.length>2)
        {
            var b2 = pBits.shift()
            var b1 = pBits.shift()

            if (b1 == b2) {
                this.debug("2 identical bits " + b1 + "is forbidden in Manchester at position" + cnt*2 )
                return null
            } else if (!b1) {
                r |= 1 
            }
           // console.log(r.toString(2) + "cnt:"+cnt )
            if (cnt == 7  ){
                res.push(r)
                cnt=0; 
                r=0
            }else {
            r= r << 1
            cnt++
            }
        }
        return res
    }
    
    
    
    
    
    /* Manchester Decoder (LSB) 
    * @pPulses : Array of Pulses
    * @pClockAligned: 
    * @pFirstPulse: 
    * @pPulseMid : (Min+  Max )/2  
    * Returns: Array of decoded bits 
    */
    decodeManchester(pClockAligned, pFirstPulse, pPulseMid, pLength) {
        var res = new Array();
        
        var next_byte = 0;
        var i_bit = pFirstPulse ? 0x80 : 0x00;
        var i = 0, i_count = 0, i_bits = 0;

        this.position--;
        this.trace[this.position]= 210; 
        



        // If first pulse is aligned to clock, it should be a short pulse
        // If pclockaligned is true, we skip one pulse
        if (pClockAligned && this.trace[this.position++] > pPulseMid){
            this.debug("ERR:decodeManchester: If first pulse is aligned to clock, it should be a short pulse  ")
            return null; 
        }
        
        
        do {
            next_byte = (next_byte >> 1) | i_bit;
            this.debug("** pos"+this.position+" pulse: " + this.trace[this.position] +"ms next_byte:"+next_byte +" ibit:"+i_bit );
            
            //We have the 8 bits
            if (++i_bits == 8) {
                res.push(next_byte);
                next_byte = i_bits = 0;
            }
            
            // Short pulses preserve the bit output value
            if (this.trace[this.position++] < pPulseMid) {
                if (this.trace[this.position++] > pPulseMid) {
                    this.debug("decodeManchesterLSB: short + long should never happed => pulse #" + (this.position -2))
                    return null;
                }
                i += 2;
            } else {
                i_bit ^= 0x80; //  Flib  bit(xor)
                i++;
            }
            //debug("i=" +i) 
        } while (i < this.trace.length);
        
        if (i_bits != 0) {
            res.pop();
            res.push(next_byte & 0xff);
        }
        return res;
    }
    
    
    readNibble(pPulses, pOffset, pNibble) {
        var out = new Uint8Array(3);
        var u32_return = 0;
        var i;
        
        u32_return = 0;
        for (i = 0; i < pNibble; i++) {
            u32_return <<= 4;
            if (pOffset & 1) {
                u32_return |= pPulses[pOffset / 2] & 0xf;
            } else {
                u32_return |= pPulses[pOffset / 2] >> 4;
            }
            pOffset++;
        }
        return u32_return;
    }
    
    
    static toHexString(byteArray) {
        var s = '';
        if (byteArray != null ) {
            byteArray.forEach(function (byte) {
            if (byte < 0x10) s += '0';
                s += ((byte & 0xFF).toString(16)).slice(-2) + " ";
            });
            s=s.toUpperCase();
        }
    return s
    }
    
    
    
    
    /*
    * pTrace: Read Pulse Debug trace 
    * Returns : Uint16Array Array of pulses  or null on decoding error
    */
    static debugToTrace(pTrace) {
        var t1 = pTrace.match(/.*\(uSec\)=(.*);/);
        if (t1 == null) {
            //alert("decodeStringPulses: Input string should contain 'Pulses(uSec)='");
            return null;
        }
        
        var t2 = t1[1].split(/,/);
        if (t2.length < 1) {
            //alert("decodeStringPulses: Input string should contains at least one pulse.");
            return null;
        }
        
        var ret = new Uint16Array(t2.length);
        
        for (var i = 0; i < t2.length; i++) {
            ret[i] = t2[i];
        }
        return ret;
    }
    
}

function Signal_selftest(){

    var demo_traces="20;188;DEBUG;Pulses=301;Pulses(uSec)=210,210,210,200,210,210,210,200,220,200,210,210,210,210,210,200,210,410,630,620,210,210,210,210,420,410,210,200,420,410,220,200,220,200,210,210,210,200,210,210,210,200,210,210,420,410,210,210,210,200,220,200,210,210,420,200,210,210,210,410,210,200,220,200,210,210,210,210,210,200,210,210,420,200,210,420,210,200,420,210,210,200,210,210,210,200,210,420,420,200,210,420,420,200,210,410,420,420,210,200,420,210,210,210,210,410,210,200,420,420,210,200,210,210,420,410,210,210,210,200,420,410,220,200,210,210,420,200,210,420,210,200,420,210,210,410,420,210,210,410,420,410,210,210,420,410,420,200,220,200,210,420,210,200,210,210,210,200,210,210,210,210,210,200,210,210,210,200,210,210,420,410,210,210,210,200,220,200,210,210,210,200,220,200,210,200,220,200,210,210,210,200,220,200,210,210,210,200,210,210,210,200,220,200,210,210,210,200,210,210,210,200,220,200,210,210,210,200,210,210,210,200,220,200,210,210,210,210,210,200,210,210,210,200,210,210,420,200,210,420,210,200,420,420,210,200,210,210,210,200,210,210,210,210,210,200,210,210,210,200,220,200,210,210,210,200,210,210,210,210,210,200,210,210,420,410,210,210,210,200,420,410,420,420,210,200,210,210,420,410,210,210,420,200,210,210,210,200,220,200,210;";
    var dek = new Signal()

    console.log("Start test");
    var trace = "20;FD;DEBUG;Pulses=301;Pulses(uSec)=210,210,210,200,220,200,210,210,210,200,220,200,210,210,210,200,210,410,630,630,210,200,210,210,420,410,210,200,420,420,210,210,210,200,210,210,210,200,210,210,210,200,210,210,420,410,220,200,210,210,210,200,210,210,420,200,210,210,210,410,210,210,210,210,210,200,210,210,210,200,210,210,420,200,220,410,210,210,410,210,210,200,210,210,210,200,220,410,420,210,210,410,420,200,210,420,410,420,210,210,420,200,210,210,210,410,210,200,420,420,210,210,210,200,420,410,210,210,210,200,420,420,210,200,220,200,420,200,220,410,210,200,420,210,210,410,420,210,210,410,420,410,210,210,420,410,420,210,210,200,210,420,210,200,210,210,210,200,210,210,210,210,210,200,220,200,210,200,220,200,420,410,220,200,210,210,210,200,220,200,210,200,220,200,210,210,210,200,210,210,210,210,210,200,210,210,210,200,210,210,210,200,220,200,210,210,210,210,210,200,210,210,210,200,210,210,210,200,220,200,210,210,210,200,220,200,210,210,210,200,210,210,210,200,210,210,420,210,210,410,210,210,410,420,210,200,210,210,210,210,210,200,210,210,210,200,220,200,210,210,210,200,210,210,210,210,200,210,210,210,210,200,220,200,420,410,210,210,210,200,420,420,420,410,210,200,210,210,420,410,210,210,420,200,220,200,210,200,220,200,210;";

    var t1 = Signal.debugToTrace(trace) 
    if ( t1 == null) {
        this.debug("Abnormal null result")
        return;
    }

    dek.setTrace(t1)
    dek.setShortPulseDuration(210);
    var nextbytes = dek.readBytes(3)
    dek.debug("> " + Signal.toHexString(nextbytes) )
    dek.debug("> Pos" + dek.getPosition() )

    var nextbytes = dek.readBytes(30)
    dek.debug("> " + Signal.toHexString(nextbytes) )

    var b1 = Signal.bytesToBits(nextbytes)
    b1.unshift(false)

    var decoded = dek.decodeManchester2(b1)
    dek.debug("> " + Signal.toHexString(decoded) )




}

Signal_selftest();
