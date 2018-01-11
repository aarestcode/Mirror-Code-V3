function error = ProgramMirrorMCU(input_file, port, CodeID)
%% CREDITS
%
% * Author: Thibaud Talon
% * Email: ttalon@caltech.edu
%
%% INPUTS/OUTPUTS
%
% * intelHEX_file = path of the Intel HEX file as a string
% * port = name of the port where the Mirror board is connected
% * error = 0 (no error)
%          -1 (Not Intel HEX file)
%          -2 (Cannot open Intel HEX file)
%          -3 (Cannot create/read Binary file)
%          >0 (Index of the line where the checksum is wrong)

error = 0;

%% Intel HEX
% Check for the extension of the file
if(strcmp(input_file(end-3:end),'.hex')==0 && strcmp(input_file(end-3:end),'.bin')==0)
    error = -1;
    return;
end

%% HEX
if(strcmp(input_file(end-3:end),'.hex'))
    % Read the Intel HEX file
    IntelHEXID = fopen(input_file,'r');
    if (IntelHEXID == -1)
        error = -2;
        return;
    end
    CodeHEX_text = textscan(IntelHEXID, '%s');
    CodeHEX_text = CodeHEX_text{1};
    fclose(IntelHEXID);
    
    
    % Create binary file
    Binary_file = input_file;
    Binary_file(end-3:end) = '.bin';
    BinaryID = fopen(Binary_file, 'w');
    if (BinaryID == -1)
        error = -3;
        return;
    end
    fclose(BinaryID);
    BinaryID = fopen(Binary_file, 'a');
    if (BinaryID == -1)
        error = -3;
        return;
    end
    
    % Write the Binary file
    N_lines = length(CodeHEX_text);
    Nbytes = 0;
    for II=1:N_lines
        line = CodeHEX_text{II};
        datalength = hex2dec(line(2:3));
        Nbytes = Nbytes + datalength;
        
        % CHECKSUM
        sum = 0;
        for III=0:4+datalength
            sum = sum + hex2dec(line(2+2*III:3+2*III));
        end
        sum_hex = dec2hex(sum);
        if(strcmp(sum_hex(end-1:end),'00')==0)
            error = II;
            break;
        end
        
        % SAVE DATA ONLY
        datatype = hex2dec(line(8:9));
        if (datatype == 0)
            for III=0:datalength-1
                fwrite(BinaryID,hex2dec(line(10+2*III:11+2*III)));
            end
        end
        
    end
    fclose(BinaryID);
else
    Binary_file = input_file;
end
%% Connect to board
[ReceievedCommand,ReceivedData,ReceivedError,error] = SendCommand(port, 255, 123);

if (ReceievedCommand ~= 255)
    disp('Wrong received command');
    error = -4;
    return;
end

if (ReceivedData == 123)
    code = 0;
    disp('Application mode detected');
elseif (ReceivedData == 1)
    code = 1;
    disp('Bootloader mode detected');
else
    disp('Wrong received value');
    error = -5;
    return;
end

%% Send code
BinaryID = fopen(Binary_file, 'r');
if (BinaryID == -1)
    error = -3;
    return;
end
Code = fread(BinaryID);
fclose(BinaryID);

PAGESIZE = 256;

if (code == 1) % Bootloader
%     disp('Start sending code to bootloader');
%     Npages = ceil(Nbytes / PAGESIZE);
%     checksum = 255 - mod(Npages+22, 255);
%     
%     % Send request
%     fwrite(s, 22, 'uint8');
%     fwrite(s, 0, 'uint8');
%     fwrite(s, 0, 'uint8');
%     fwrite(s, 0, 'uint8');
%     fwrite(s, Npages, 'uint8');
%     fwrite(s, checksum, 'uint8');
%     index = 1;
%     for II=1:Npages
%         
%         [R, count] = fread(s, 6, 'uint8');
%         if (count == 0)
%             disp('Nothing returned');
%             error = -6;
%             fclose(s);
%             return;
%         end
%         if (R(5) ~= 0)
%             disp(R);
%             fclose(s);
%             error = -5;
%             return;
%         end
%         
%         while (index <= min(II*PAGESIZE, Nbytes) )
%             fwrite(s, Code(index), 'uint8');
%             %disp([num2str(index),' -> ', dec2hex(Code(index),2)]);
%             index = index + 1;
%         end
%         
%         if (II == Npages)
%             for III=index:PAGESIZE*Npages
%                 fwrite(s, 255, 'uint8');
%                 %disp([num2str(index),' -> ', dec2hex(255,2)]);
%                 index = index + 1;
%             end
%         end
%         
%         disp(['Page ', num2str(II), '/', num2str(Npages), ' loaded']);
%     end
    
    
elseif (code == 0)
    % Application code
    disp('Start sending code to application');
    
    Npages = ceil(Nbytes / PAGESIZE);
    [ReceievedCommand,ReceivedData,ReceivedError,error] = SendCommand(port, 245, bitsll(CodeID,16) + Npages);
    
    index = 1;
    for II=1:Npages
        
        if (error)
            disp('Nothing returned');
            error = -6;
            return;
        end
        if (ReceivedError ~= 0)
            disp('Error returned');
            error = -5;
            return;
        end
        
        Page = 255*ones(PAGESIZE,1);
        II_Page = 1;
        
        while (index <= min(II*PAGESIZE, Nbytes) )
            Page(II_Page) = Code(index);
            index = index + 1;
            II_Page = II_Page +1;
        end
                
        [ReceievedCommand,ReceivedData,ReceivedError,error] = SendPage(port, Page);
        
        disp(['Page ', num2str(II), '/', num2str(Npages), ' loaded']);
    end
end

if (error)
    disp('Nothing returned');
    error = -6;
    return;
end
if (ReceivedError ~= 0)
    disp('Error returned');
    error = -5;
    return;
end
error = 0;