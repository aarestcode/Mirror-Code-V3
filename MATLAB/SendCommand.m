function [ReceievedCommand,ReceivedData,ReceivedError,error] = SendCommand(port, Command, Data)

%% Connect to board
s = serial(port,'BaudRate', 9600, 'Timeout', 10);
fopen(s);

try
    %% Calculate checksum
    if (Data >= 0)
        Data_hex = dec2hex(Data, 8);
    else
       Data_hex = dec2hex(2^32 + Data, 8); 
    end
    Sum = Command + hex2dec(Data_hex(1:2)) + hex2dec(Data_hex(3:4)) + hex2dec(Data_hex(5:6)) + hex2dec(Data_hex(7:8));
    checksum = 255 - mod(Sum, 256);
    
    %% SEND
    fwrite(s, Command, 'uint8');
    fwrite(s, hex2dec(Data_hex(1:2)), 'uint8');
    fwrite(s, hex2dec(Data_hex(3:4)), 'uint8');
    fwrite(s, hex2dec(Data_hex(5:6)), 'uint8');
    fwrite(s, hex2dec(Data_hex(7:8)), 'uint8');
    fwrite(s, 0, 'uint8');
    fwrite(s, checksum, 'uint8');
    fprintf('Sent     | Command = %i, Data = %li (0x%s)\n', Command, Data, Data_hex);
        
    %% RECEIVE
    [R, count] = fread(s, 7, 'uint8');
    if (count < 7)
        fclose(s);
        error = 1;
        return;
    end
    
    Sum = R(1) + R(2) + R(3) + R(4) + R(5) + R(6) + R(7);
    if (mod(Sum,256) ~= 255)
        disp('Wrong received checksum');
        disp(R);
        fclose(s);
        error = 2;
        return;
    end
    
    ReceievedCommand = R(1);
    ReceivedData = R(2)*2^(3*8) + R(3)*2^(2*8) + R(4)*2^(8) + R(5);
    ReceivedData_hex = dec2hex(ReceivedData,8);
    ReceivedError = R(6);
    
    fprintf('Received | Command = %i, Data = %li (0x%s), Error = %i\n', ReceievedCommand, ReceivedData, ReceivedData_hex, ReceivedError);
    fclose(s);
    error = 0;
catch
    ReceievedCommand = 0;
    ReceivedData = 0;
    ReceivedError = 0;
    error = -1;
    fclose(s);
end