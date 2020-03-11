
class Dekoder {


    position = 0;                    // Current position of decoder
    decoded = "";               // Decoded string
    shortPulseDuration = 150;    // Duration of short pulse
    pulses;                     // Array of pulses

    constructor() {
    }

    reset() {
        this.position = 0;
    }


    getPosition() {
        return this.position;
    }

    getDecoded() {
        return this.decoded;
    }

    setShortPulseDuration(pMicrosec) {
        this.shortPulseDuration = pMicrosec;
    }

    setPulses(pPulses) {
        this.pulses = pPulses;
    }

    getPulses() {
        return this.pulses;
    }


    readBytes(pNumBytes) {
        var ret = [];
        var bitcount = 0;
        var u8val = 0;
        var state = 0;
        while (ret.length < pNumBytes) {
            state = ((this.position % 2) == 0) ? "1" : "0"
            var tCnt = Math.floor(this.pulses[this.position] / this.shortPulseDuration)

            for (var k = 0; k < tCnt; k++) {
                u8val = u8val << 1;
                if (state == "1") {
                    u8val |= 1
                }
                bitcount++;
                if (bitcount == 8) {
                    bitcount = 0;
                    ret.push(u8val)
                    console1("Push:" + u8val)
                    u8val = 0
                }
            }

            this.position++
            console1("Position:" + this.position)
        }
        return ret;
    }

  






    /* Manchester Decoder (LSB) 
     * @pPulses : Array of Pulses
     * @pClockAligned: 
     * @pFirstPulse: 
     * @pPulseMid : (Min+  Max )/2  
     * Returns: Array of decoded bits 
     */
    decodeManchester(pClockAligned, pFirstPulse, pPulseMid) {
        var res = new Array();

        var u16 = 0;
        var i_bit = pFirstPulse ? 0x80 : 0x00;
        var i = 0, i_count = 0, i_bits = 0;
        do {
            u16 = (u16 >> 1) | i_bit;
            //console1("** pulse: " + pPulses[pos] +"u16:"+u16 +" ibit:"+i_bit );

            if (++i_bits == 8) {
                res.push(u16);
                u16 = i_bits = 0;
            }

            if (this.pulses[this.position++] < pPulseMid) {
                if (this.pulses[this.position++] > pPulseMid) {
                    console1("decodeManchesterLSB: short + long ERROR Pulse #" + pos)
                    return null;
                }
                i += 2;
            } else {
                i_bit ^= 0x80;
                i++;
            }
            //console1("i=" +i) 
        } while (i < this.pulses.length);

        if (i_bits != 0) {
            res.pop();
            res.push(u16 & 0xff);
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


    toHexString(byteArray) {
        var s = '';
        byteArray.forEach(function (byte) {
            if (byte < 0x10) s += '0';
            s += ((byte & 0xFF).toString(16)).slice(-2) + " ";
        });
        return s.toUpperCase();
    }




    /*
    * Read Pulse Debug trace and return a Uint16Array with the pulses
    * nb: Function returns null on decoding error 
    */
    initFromTrace(pTrace) {
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

        this.pulses = new Uint16Array(t2.length);
        //console1("decodeStringPulses: " + t2.length + " tokens found.")
        for (var i = 0; i < t2.length; i++) {
            this.pulses[i] = t2[i];
        }
        return this.pulses;
    }


}