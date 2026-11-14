export class JsonLines {
  constructor(onPacket) { this.onPacket=onPacket; this.pending=''; this.discard=false; }
  push(text) {
    for(const part of text.split(/(?<=\n)/)) {
      if(!this.discard)this.pending+=part;
      if(this.pending.length>65536){this.pending='';this.discard=true;}
      if(part.endsWith('\n')) {
        let packet;
        if(!this.discard)try{packet=JSON.parse(this.pending);}catch{/* Boot messages/partial first line. */}
        this.pending='';this.discard=false;
        if(packet&&typeof packet==='object')this.onPacket(packet);
      }
    }
  }
}

export class SerialConnection {
  constructor(onPacket,onEnd) { this.onPacket=onPacket;this.onEnd=onEnd;this.stopped=false; }
  async open(serial) {
    const port=await serial.requestPort();
    if(this.stopped)return;
    await port.open({baudRate:460800,bufferSize:65536});
    this.port=port;
    if(this.stopped){await port.close();return;}
    // Native USB CDC firmware uses DTR to detect an open host connection.
    try{await port.setSignals({dataTerminalReady:true,requestToSend:false});}
    catch(e){await port.close();throw e;}
    if(this.stopped){await port.close();return;}
    this.task=this.read();
  }
  async read() {
    let failure;
    try {
      if(!this.port.readable)throw Error('The selected port cannot be read.');
      this.reader=this.port.readable.getReader();
      const decoder=new TextDecoder(),lines=new JsonLines(this.onPacket);
      while(!this.stopped) {
        const {value,done}=await this.reader.read();
        if(done)break;
        if(!this.stopped)lines.push(decoder.decode(value,{stream:true}));
      }
    }catch(e){failure=e;}
    finally {
      this.reader?.releaseLock();this.reader=null;
      try{await this.port.close();}catch(e){failure ||= e;}
      this.onEnd(this.stopped?null:(failure||Error('USB disconnected. Connect the board again.')));
    }
  }
  async close() {
    this.stopped=true;
    try{await this.reader?.cancel();}catch{/* Device may already be unplugged. */}
    await this.task;
  }
}
