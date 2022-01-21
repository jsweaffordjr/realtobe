% creates animation of data
load('with_noise.mat');
record = 1; % set to 1 to record video

pausetime = 0.02; % time between frames
initialpause = 0; % pause length before animation begins

%setup video
if (record == 1)
    writerObj = VideoWriter('WithNoise.avi');
    writerObj.FrameRate = 50;
    open(writerObj);
end

N = length(t);

for k = 1:N
    figure(1)
    %Plot IMU orientation at timestep k
    subplot('Position', [0.1,0.1,0.85,0.25])
    cla
    plot(t(1:k),f(1:k),'c') % known function
    hold on
    plot(t,fdot,'r','LineWidth',3) % known derivative
    plot(t(1:k),c(1:k),'kx') % homogeneous differentiator
    axis([0 10 -2 2])
    xlabel('Time')
    legend('sin(x)','cos(x)','Homogeneous diff.')
    
    subplot('Position', [0.1,0.4,0.85,0.25])
    plot(t(1:k),f(1:k),'c') % known function
    hold on
    plot(t,fdot,'r','LineWidth',3) % known derivative
    plot(t(1:k),b(1:k),'gx') % Savitky-Golay filter
    axis([0 10 -2 2])
    xlabel('Time')
    legend('sin(x)','cos(x)','Savitzky-Golay')
    
    subplot('Position', [0.1,0.7,0.85,0.25])
    plot(t(1:k),f(1:k),'c') % known function
    hold on
    plot(t,fdot,'r','LineWidth',3) % known derivative
    plot(t(1:k),a(1:k),'bx') % finite difference
    axis([0 10 -2 2])
    legend('sin(x)','cos(x)','Finite difference')
    
    title('Real-time Differentiation Methods (with noise), dt = 0.02 sec')
    xlabel('Time')
    
%     grid on

    %Set figure size
    set(gcf,'Position', [150, 80, 650, 700]);

    %write vid file
    if (record == 1)
        frame = getframe(gcf);
        writeVideo(writerObj,frame);
    end

end

if (record == 1)
    close(writerObj);
end
