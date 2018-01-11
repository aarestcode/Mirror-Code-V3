function [ReceievedCommand,ReceivedData,ReceivedError,error] = SendPage(port, Page)

%% Connect to board
s = serial(port,'BaudRate', 9600, 'Timeout', 10);
fopen(s);

try
    %% SEND
    for II = 1:length(Page)
        fwrite(s, Page(II), 'uint8');
    end
    fprintf('Sent     | %i bytes\n', length(Page));
    
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
catch ME
    fprintf('Critical error: %s\n', ME.identifier);
    ReceievedCommand = 0;
    ReceivedData = 0;
    ReceivedError = 0;
    error = -1;
    fclose(s);
end