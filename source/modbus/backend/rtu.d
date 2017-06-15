///
module modbus.backend.rtu;

import modbus.backend.base;

///
class RTU : BaseBackend!256
{
protected:
    enum lengthOfCRC = 2;

public:
    ///
    this(Connection c, SpecRules s=null) { super(c, s, lengthOfCRC, 0); }

override:
    ///
    void start(ulong dev, ubyte func) { appendDF(dev, func); }

    ///
    void send()
    {
        scope (exit) idx = 0;
        append(cast(const(void)[])(crc16(buffer[0..idx])[]));
        conn.write(buffer[0..idx]);
        version (modbusverbose)
            .trace("write bytes: ", buffer[0..idx]);
    }

    ///
    Response read(size_t expectedBytes)
    {
        auto res = baseRead(expectedBytes);
        if (!checkCRC(res.data))
            throw checkCRCException(res.dev, res.fnc);
        res.data = res.data[devOffset+sr.deviceTypeSize+functionTypeSize..$-lengthOfCRC];
        return res;
    }
}

unittest
{
    import std.algorithm;
    import std.bitmanip;

    void[] buf;

    auto rtu = new RTU(new class Connection
    { override:
        void write(const(void)[] t) { buf = t.dup; }
        void[] read(void[] buffer)
        {
            assert(buffer.length <= buf.length);
            buffer[0..buf.length] = buf[];
            return buffer[0..buf.length];
        }
    });

    enum C1 = ushort(10100);
    enum C2 = ushort(12345);
    rtu.start(1, 6);
    rtu.append(C1);
    rtu.append(C2);
    assert(!rtu.messageComplite);
    assert(rtu.tempBuffer.length == 2 + 2 + 2);
    rtu.send();
    assert(rtu.messageComplite);
    assert(rtu.tempBuffer.length == 0);
    assert(equal(cast(ubyte[])buf[0..$-2],
                cast(ubyte[])[1, 6] ~ nativeToBigEndian(C1) ~ nativeToBigEndian(C2)));

    auto crc = cast(ubyte[])crc16(buf[0..$-2]);
    assert(crc[0] == (cast(ubyte[])buf)[$-2]);
    assert(crc[1] == (cast(ubyte[])buf)[$-1]);
}

/++ Check CRC16 of data
    Params:
    data = last two bytes used as CRC16
 +/
bool checkCRC(const(void)[] data) pure nothrow @trusted @nogc
{
    auto msg = cast(const(ubyte[]))data;
    auto a = msg[$-2..$];
    auto b = crc16(msg[0..$-2]);
    return a[0] == b[0] && a[1] == b[1];
}

@safe unittest
{
    immutable ubyte[] d1 = [0x02, 0x03, 0x00, 0x00, 0x00, 0x05, 0x85, 0xFA];
    immutable ubyte[] d2 = [0x01, 0x04, 0x02, 0xFF, 0xFF, 0xB8, 0x80];
    immutable ubyte[] d3 = [0x01, 0x04, 0x02, 0xFF, 0xFF, 0xB8, 0x00]; // invalid frame

    assert(d1.checkCRC);
    assert(d2.checkCRC);
    assert(!d3.checkCRC);
}

/// Initial crc value should be 0xFFFF
private void crc16(ref ushort crc, in ubyte[] buff) pure nothrow @safe @nogc
{
    enum ushort poly = 0xA001; // ModBus CRC polynomic
    enum ushort lsb = 1;

    foreach(const ubyte b; buff)
    {
        crc ^= b;

        for(byte i = 0; i < 8; i++)
        {
            const bool lsbIsSet = (crc & lsb) != 0;

            crc >>>= 1;

            if(lsbIsSet)
                crc ^= poly;
        }
    }
}

unittest
{
    {
        ushort crc = 0xFFFF;
        crc16(crc, [1]);
        assert(crc == 0x807E);
    }
    {
        ushort crc = 0xFFFF;
        crc16(crc, [2]);
        assert(crc == 0x813E);
    }
    {
        ushort crc = 0xFFFF;
        crc16(crc, ['W', 'Z']);
        assert(crc == 0xBBBF);
    }
}

/++ Calculate CRC16
    Params:
    data = input data for calculation CRC16
 +/
ubyte[2] crc16(const(void)[] data) pure nothrow @trusted @nogc
{
    ushort crc = 0xFFFF;
    crc16(crc, cast(const ubyte[]) data);

    import std.bitmanip;

    return nativeToLittleEndian(crc);
}

@safe unittest
{
    immutable ubyte[] data = [0x02, 0x03, 0x00, 0x00, 0x00, 0x05];
    assert(crc16(data)[1] == 0xFA);
    assert(crc16(data)[0] == 0x85);
}
