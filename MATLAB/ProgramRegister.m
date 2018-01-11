function error = ProgramRegister(port, RegisterCSV)

%% Read CSV
M = csvread(RegisterCSV);
N = size(M,1);

%% Send
for II = 1:N
    error = SendCommand(port,II,M(II));
    if (error == 0)
       disp(['Register value ', num2str(II), '/', num2str(Npages), ' loaded']); 
    end
end

%% Save Register
error = SendCommand(port,225,0);

end