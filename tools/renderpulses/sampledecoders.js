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


function decode_tfa(){

}