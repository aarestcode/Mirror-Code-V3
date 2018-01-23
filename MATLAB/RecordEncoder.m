function [time, encoder_data] = RecordEncoder(port, encoderID, duration_s)

figure;

encoder_data = [];
time = [];

tic;
t = toc;

while toc<duration_s
    t = toc;
    [~,ReceivedData,~,error] = SendCommand(port, 176+10*(encoderID), 0);
   
    if (error == 0)
        time = [time, t];
        encoder_data = [encoder_data, ReceivedData];
        plot(time, encoder_data, 'k-', 'Linewidth',2); 
        drawnow 
    end
end



end