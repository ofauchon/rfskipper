

/* CONSTS */

/*
const MAX_PULSES = 512;
const RAWSIGNAL_SAMPLE_RATE = 1;
//    const OREGON_PulseLength = 41;
const PROTO_PULSEMIN = 200 / RAWSIGNAL_SAMPLE_RATE;
const PROTO_PULSEMAX = 420 / RAWSIGNAL_SAMPLE_RATE;
const PROTO_PULSEMID = ((PROTO_PULSEMIN + PROTO_PULSEMAX) / 2);
*/



/* Manchester Decoder (LSB) 
 * @pPulses : Array of Pulses
 * @pClockAligned: 
 * @pFirstPulse: 
 * @pPulseMid : (Min+  Max )/2  
 * Returns: Array of decoded bits 
 */
function decodeManchester(pPulses, pClockAligned, pFirstPulse, pPulseMid) {
    var pos = 0;
    var res = new Array();

    var u16 = 0;
    var i_bit = pFirstPulse ? 0x80 : 0x00;
    var i = i_count = i_bits = 0;
    do {
        u16 = (u16 >> 1) | i_bit;
        //console1("** pulse: " + pPulses[pos] +"u16:"+u16 +" ibit:"+i_bit );

        if (++i_bits == 8) {
            res.push(u16);
            u16 = i_bits = 0;
        }

        if (pPulses[pos++] < pPulseMid) {
            if (pPulses[pos++] > pPulseMid) {
                console1("decodeManchesterLSB: short + long ERROR Pulse #" + pos)
                return null;
            }
            i += 2;
        } else {
            i_bit ^= 0x80;
            i++;
        }
        //console1("i=" +i) 
    } while (i < pPulses.length);

    if (i_bits != 0) {
        res.pop();
        res.push(u16 & 0xff);
    }
    console1("decodeManchester: result => " + toHexString(res));
    return res;
}




function readNibble(pPulses, pOffset, pNibble) {
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


function toHexString(byteArray) {
    var s = '';
    byteArray.forEach(function (byte) {
        if (byte < 0x10) s += '0';
        s += ((byte & 0xFF).toString(16)).slice(-2) + " ";
    });
    return s.toUpperCase();
}