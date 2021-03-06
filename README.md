### Modbus protocol

Library provides modbus wrapper over RTU and TCP connections.
By default using `serialport` package.

Simple usage:

```d
auto mbus = new ModbusRTU(new SerialPort("/dev/ttyUSB0", 19200));

mbus.writeTimeout = 100.msecs;
mbus.readTimeout = 2.seconds;
mbus.readFrameGap = 5.msecs; // use for detect end of data pack
```

For tcp connection using `std.socket.TcpSocket`.

```d
auto addr = "device_IP";
ushort port = 502; // or 503
auto mbs = new ModbusTCP(new InternetAddress(addr, port));
writeln(mbs.readInputRegisters(1, 17, 1));
```

`ModbusRTU` and `ModbusTCP` close serial port and socket in destructors.

You can configure library with custom serialport realization.
For this past `subConfiguration "modbus" "custom"` to your `dub.sdl`
or `"subConfigurations": { "modbus": "custom" }` to your `dub.json`.
In this case `Modbus` don't manage your serial port or tcp connection.
They uses through simple interfaces with `read` and `write` methods and
you must close opened connections by yourself.

Example:

```d
import myserialport;
import modbus;
import modbus.backend;

auto com = new MySerialPort();

auto mbus = new Modbus(new RTU(new class Connection{
            override:
                void write(const(void)[] msg) { com.write(msg); }
                void[] read(void[] buffer) { return com.read(buffer); }
            }));

auto registers = mbus.readInputRegisters(device, address, count);
```