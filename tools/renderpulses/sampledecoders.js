var sampledecoders=['ticpulses', 'tfa']

function decode_ticpulses( trace){

    // Use a New Signal Instance 
    var dek = new Signal()
    dek.setTrace(trace)
    dek.setPosition(0);
    dek.setShortPulseDuration(210);

    var tbytes = dek.readBytes(0xFF) // Read max 0xFF bytes
    console1("3 first bytes " + Signal.toHexString(tbytes.slice(0,3)))
    
    var tbits = Signal.bytesToBits(tbytes.slice(3)) 
    //console1("Next 16 bits" + Signal.toHexString(tbits.slice(0,16)))

    var payload = dek.decodeManchester2(tbits)
    payload = payload.slice(1) // Skip first bit : packet length
    console1("Next Manchester Bytes"+  Signal.toHexString(payload));
    var id = payload[1]

    id = (id * 0x100) + payload[5]
    id = (id * 0x100) + payload[4]
    id = (id * 0x100) + payload[3]
    id = (id * 0x100) + payload[2]

    var cntr = payload [10]
    cntr = (cntr* 0x100)+ payload[9]
    cntr = (cntr* 0x100) + payload[8]
    cntr = (cntr* 0x100) + payload[7]

    console1("Cartelectronique : MeterID: " + id + " Counter: " + cntr)

}


function decode_tfa(trace){
    var base_width = 500
    var short_width = 2000
    var long_width  = 4000
    var error_pct=0.2 // 20% error
    console1("DECODE TFA with trace of " + trace.length + " elements")

    var nibbles=[]
    var pos=0; 
    var nibble=0;
    var bitcount=0
    while (pos < trace.length){
        if (!(trace[pos] > base_width * (1-error_pct) && trace[pos] < base_width * (1+error_pct))){
            pos++
            continue
        }
        
        if (trace[pos+1] > short_width * (1-error_pct) && trace[pos+1] < short_width * (1+error_pct)){
            nibble = nibble *2 
            bitcount++
            pos++
            //console1("Short pulse => bit 0 @" +pos +" nibble="+ nibble)
        } else if (trace[pos+1] > long_width * (1-error_pct) && trace[pos+1] < long_width * (1+error_pct)){
                nibble = nibble *2 +1
                bitcount++
                pos++
              //  console1("Long pulse =>  bit 1 @"  + pos  +" nibble="+ nibble)
            } else {
                console1("Err: Base pulse not followed by short/long @" + pos)
        }
        if(bitcount==4){
                console1("New Nibble :" + nibble.toString(16))
                nibbles.push(nibble)
                nibble=0
                bitcount=0
        }
        pos++
         
    }

    var pType = nibbles[0]
    var pId = ((nibbles[1] & 0x0F) << 4 ) | nibbles[2]
    var pBat = nibbles[3] & 0x08
    var pButton = ((nibbles[3] & 0x04) >>2)
    var pTemp = nibbles[4]
    pTemp = (pTemp << 4) + nibbles[5]
    pTemp = (pTemp << 4) + nibbles[6]
    
    
     
    console1("type:"+ pType + " id:" + pId + " bat:" + pBat + " button:" + pButton + " temp:" + pTemp  )


}