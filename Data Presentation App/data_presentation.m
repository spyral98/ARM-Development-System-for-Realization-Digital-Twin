clear all;
close all;
clc;

try
    L = 512; % Length of the ADC device buffer
    N = 100; % How many buffers the adc plot show
    adc_plot_length = N*L; % Total number of elements that the adc plot shows
    id = "u";
    
    L_imu = 64; % Length of the IMU device buffer
    N_i = 20; % How many buffers the IMU plot show 
    imu_length = N_i*64; % Total number of elements that the IMU plot shows 
    
    i = 0; % Microphone plot iteration counter
    j = 0; % Current plot iteration counter
    k = 0; % IMU plot iteration counter
    
    % --Time domain plots--
    f1=figure(1);
    set(f1,'name','Time Domain Plots', 'Position', [750 50 1000 1200]);
    
    % Preallocating data arrays
    mic_data = zeros([1,adc_plot_length]);
    cur_data = zeros([1,adc_plot_length]);
    imu_data = zeros([1,6*L_imu]);
    accel_x_data = zeros([1,imu_length]);
    accel_y_data = zeros([1,imu_length]);
    accel_z_data = zeros([1,imu_length]);
    gyr_x_data = zeros([1,imu_length]);
    gyr_y_data = zeros([1,imu_length]);
    gyr_z_data = zeros([1,imu_length]);
    
    % Creating the split plots
    mic_plot = subplot(4,2,1);
    cur_plot = subplot(4,2,2);
    acc_x_plot = subplot(4,2,3);
    acc_y_plot = subplot(4,2,5);
    acc_z_plot = subplot(4,2,7);
    gyr_x_plot = subplot(4,2,4);
    gyr_y_plot = subplot(4,2,6);
    gyr_z_plot = subplot(4,2,8);
    
    % Plotting the data as circular buffer
    plot(mic_plot, [ mic_data(i*L+1:end)  mic_data(1:i*L)] );
    ylim(mic_plot, [0,3.3]);
    title(mic_plot, "Microphone");
    ylabel(mic_plot, "Voltage [V]");
    xlabel(mic_plot, "Samples");
    
    plot(cur_plot, [ cur_data(i*L+1:end)  cur_data(1:i*L)] );
    ylim(cur_plot, [-0.5,3]);
    title(cur_plot, "Motor Current");
    ylabel(cur_plot, "Current [A]");
    xlabel(cur_plot, "Samples");
    
    plot(acc_x_plot, [accel_x_data(k+1:end) accel_x_data(1:k)]);
    ylim(acc_x_plot, [-2.5,2.5]);
    title(acc_x_plot, "Acceleration - X axis");
    ylabel(acc_x_plot, "Acceleration [g]");
    xlabel(acc_x_plot, "Samples");
    
    plot(acc_y_plot, [accel_y_data(k+1:end) accel_y_data(1:k)]);
    ylim(acc_y_plot, [-2.5,2.5]);
    title(acc_y_plot, "Acceleration - Y axis");
    ylabel(acc_y_plot, "Acceleration [g]");
    xlabel(acc_y_plot, "Samples");
    
    plot(acc_z_plot, [accel_z_data(k+1:end) accel_z_data(1:k)]);
    ylim(acc_z_plot, [-2.5,2.5]);
    title(acc_z_plot, "Acceleration - Z axis");
    ylabel(acc_z_plot, "Acceleration [g]");
    xlabel(acc_z_plot, "Samples");
    
    plot(gyr_x_plot, [gyr_x_data(k+1:end) gyr_x_data(1:k)]);
    ylim(gyr_x_plot, [-5,5]);
    title(gyr_x_plot, "Angular Velocity - X axis");
    ylabel(gyr_x_plot, "Angular Velocity [rad/s]");
    xlabel(gyr_x_plot, "Samples");
    
    plot(gyr_y_plot, [gyr_y_data(k+1:end) gyr_y_data(1:k)]);
    ylim(gyr_y_plot, [-5,5]);
    title(gyr_y_plot, "Angular Velocity - Y axis");
    ylabel(gyr_y_plot, "Angular Velocity [rad/s]");
    xlabel(gyr_y_plot, "Samples");
    
    plot(gyr_z_plot, [gyr_z_data(k+1:end) gyr_z_data(1:k)]);
    ylim(gyr_z_plot, [-5,5]);
    title(gyr_z_plot, "Angular Velocity - Z axis");
    ylabel(gyr_z_plot, "Angular Velocity [rad/s]");
    xlabel(gyr_z_plot, "Samples");
    
    
    
    %Frequency domain plots
    f2=figure(2);
    set(f2,'name','Frequency Domain Plots', 'Position', [1850 50 1000 1200]);
    
    fs = 12000; % ADC sampling frequency
    freq = fs*(0:(L/2-1))/L; % ADC frequency range
    
    fs_imu = 1000; % IMU sampling frequency
    freq_imu = fs_imu*(0:(L_imu/2-1))/L_imu; % IMU frequency range
    
    % Creating the split plots 
    mic_fft_plot = subplot(4,2,1);
    cur_fft_plot = subplot(4,2,2);
    acc_x_fft_plot = subplot(4,2,3);
    acc_y_fft_plot = subplot(4,2,5);
    acc_z_fft_plot = subplot(4,2,7);
    gyro_x_fft_plot = subplot(4,2,4);
    gyro_y_fft_plot = subplot(4,2,6);
    gyro_z_fft_plot = subplot(4,2,8);
    
    % Plotting the FFT data
    mic_fft_data = zeros([1,L/2]);
    plot(mic_fft_plot, freq, mic_fft_data);
    title(mic_fft_plot, "Microphone PSD");
    ylabel(mic_fft_plot, "PSD    [dB]");
    xlabel(mic_fft_plot, "Frequency [Hz]");
    ylim(mic_fft_plot, [-30,30]);
    yticks(mic_fft_plot, -30:10:30);
    xlim(mic_fft_plot, [0,fs/2]);
    
    cur_fft_data = zeros([1,L/2]);
    plot(cur_fft_plot, freq, cur_fft_data);
    title(cur_fft_plot, "Motor Current PSD");
    ylabel(cur_fft_plot, "PSD    [dB]");
    xlabel(cur_fft_plot, "Frequency [Hz]");
    ylim(cur_fft_plot, [-30,30]);
    yticks(cur_fft_plot, -30:10:30);
    xlim(cur_fft_plot, [0,fs/2]);
    
    % Plotting of the IMU FFT data
    acc_x_fft_data = zeros([1,L_imu/2]);
    stem(acc_x_fft_plot, freq_imu, acc_x_fft_data,'filled','BaseValue',-inf, 'MarkerSize',5);
    title(acc_x_fft_plot, "Acceleration - X axis PSD");
    ylabel(acc_x_fft_plot, "PSD    [dB]");
    xlabel(acc_x_fft_plot, "Frequency [Hz]");
    ylim(acc_x_fft_plot, [-30,30]);
    yticks(acc_x_fft_plot, -30:10:30);
    xlim(acc_x_fft_plot, [0,fs_imu/2]);
    
    acc_y_fft_data = zeros([1,L_imu/2]);
    stem(acc_y_fft_plot, freq_imu, acc_y_fft_data,'filled','BaseValue',-inf, 'MarkerSize',5);
    title(acc_y_fft_plot, "Acceleration - Y axis PSD");
    ylabel(acc_y_fft_plot, "PSD    [dB]");
    xlabel(acc_y_fft_plot, "Frequency [Hz]");
    ylim(acc_y_fft_plot, [-30,30]);
    yticks(acc_y_fft_plot, -30:10:30);
    xlim(acc_y_fft_plot, [0,fs_imu/2]);
    
    acc_z_fft_data = zeros([1,L_imu/2]);
    stem(acc_z_fft_plot, freq_imu, acc_z_fft_data,'filled','BaseValue',-inf, 'MarkerSize',5);
    title(acc_z_fft_plot, "Acceleration - Z axis PSD");
    ylabel(acc_z_fft_plot, "PSD    [dB]");
    xlabel(acc_z_fft_plot, "Frequency [Hz]");
    ylim(acc_z_fft_plot, [-30,30]);
    yticks(acc_z_fft_plot, -30:10:30);
    xlim(acc_z_fft_plot, [0,fs_imu/2]);
    
    gyro_x_fft_data = zeros([1,L_imu/2]);
    stem(gyro_x_fft_plot, freq_imu, gyro_x_fft_data,'filled','BaseValue',-inf, 'MarkerSize',5);
    title(gyro_x_fft_plot, "Angular Velocity - X axis PSD");
    ylabel(gyro_x_fft_plot, "PSD    [dB]");
    xlabel(gyro_x_fft_plot, "Frequency [Hz]");
    ylim(gyro_x_fft_plot, [-30,30]);
    yticks(gyro_x_fft_plot, -30:10:30);
    xlim(gyro_x_fft_plot, [0,fs_imu/2]);
    
    gyro_y_fft_data = zeros([1,L_imu/2]);
    stem(gyro_y_fft_plot, freq_imu, gyro_y_fft_data, "filled",'BaseValue',-inf, 'MarkerSize',5);
    title(gyro_y_fft_plot, "Angular Velocity - Y axis PSD");
    ylabel(gyro_y_fft_plot, "PSD    [dB]");
    xlabel(gyro_y_fft_plot, "Frequency [Hz]");
    ylim(gyro_y_fft_plot, [-30,30]);
    yticks(gyro_y_fft_plot, -30:10:30);
    xlim(gyro_y_fft_plot, [0,fs_imu/2]);
    
    gyro_z_fft_data = zeros([1,L_imu/2]);
    stem(gyro_z_fft_plot, freq_imu, gyro_z_fft_data, "filled",'BaseValue',-inf, 'MarkerSize',5);
    title(gyro_z_fft_plot, "Angular Velocity - Z axis PSD");
    ylabel(gyro_z_fft_plot, "PSD    [dB]");
    xlabel(gyro_z_fft_plot, "Frequency [Hz]");
    ylim(gyro_z_fft_plot, [-30,30]);
    yticks(gyro_z_fft_plot, -30:10:30);
    xlim(gyro_z_fft_plot, [0,fs_imu/2]);
    
    s = serialport("COM3", 115200); % Opening serial port with the given baurate
    flush(s);
    pause(0.01);
    
    write(s, 'r', "char");
    
    while (1)
    
        % Reading plot id
        previousid = id;
        id = read(s,1,"char");
        
        % Microphone time data
        if id == 'm'
            mic_data(i*L+1:(i+1)*L) = read(s,L,"single");
            mic_plot.Children.YData = [ mic_data((i+1)*L+1:end)  mic_data(1:i*L)];
            i = mod(i+1, N);
           
            
        % Current time data
        elseif id == 'c'
            cur_data(j*L+1:(j+1)*L) = read(s,L,"single");
            cur_plot.Children.YData = [ cur_data((i+1)*L+1:end)  cur_data(1:i*L)];
            j = mod(j+1, N);
            
        % IMU time data    
        elseif id == 'i'
           imu_data = read(s,6*L_imu,"single");
           
           accel_x_data(k*L_imu+1:(k+1)*L_imu) = imu_data(1:L_imu);
           accel_y_data(k*L_imu+1:(k+1)*L_imu) = imu_data(L_imu+1:2*L_imu);
           accel_z_data(k*L_imu+1:(k+1)*L_imu) = imu_data(2*L_imu+1:3*L_imu);
           gyro_x_data(k*L_imu+1:(k+1)*L_imu) = imu_data(3*L_imu+1:4*L_imu);
           gyro_y_data(k*L_imu+1:(k+1)*L_imu) = imu_data(4*L_imu+1:5*L_imu);
           gyro_z_data(k*L_imu+1:(k+1)*L_imu) = imu_data(5*L_imu+1:6*L_imu);
    
           acc_x_plot.Children.YData = [accel_x_data((k+1)*L_imu+1:end) accel_x_data(1:k*L_imu)];
           acc_y_plot.Children.YData = [accel_y_data((k+1)*L_imu+1:end) accel_y_data(1:k*L_imu)];
           acc_z_plot.Children.YData = [accel_z_data((k+1)*L_imu+1:end) accel_z_data(1:k*L_imu)];
           gyr_x_plot.Children.YData = [gyro_x_data((k+1)*L_imu+1:end) gyro_x_data(1:k*L_imu)];
           gyr_y_plot.Children.YData = [gyro_y_data((k+1)*L_imu+1:end) gyro_y_data(1:k*L_imu)];
           gyr_z_plot.Children.YData = [gyro_z_data((k+1)*L_imu+1:end) gyro_z_data(1:k*L_imu)];
           
           k = mod(k+1, N_i);
           
        % Microphone FFT data
        elseif id == 'M'
            mic_fft_data = read(s,L/2,"single");
            mic_fft_plot.Children.YData = 10*log10(mic_fft_data);
            
        % Current FFT data
        elseif id == 'C'
            cur_fft_data = read(s,L/2,"single");
            cur_fft_plot.Children.YData = 10*log10(cur_fft_data);
            
        % IMU (Accelerometer X) FFT data    
        elseif id == '1'
            acc_x_fft_data = read(s,L_imu/2,"single");
            acc_x_fft_plot.Children.YData = 10*log10(acc_x_fft_data);
            
        % IMU (Accelerometer Y) FFT data    
        elseif id == '2'
            acc_y_fft_data = read(s,L_imu/2,"single");
            acc_y_fft_plot.Children.YData = 10*log10(acc_y_fft_data);
            
        % IMU (Accelerometer Z) FFT data    
        elseif id == '3'
            acc_z_fft_data = read(s,L_imu/2,"single");
            acc_z_fft_plot.Children.YData = 10*log10(acc_z_fft_data);
            
        % IMU (Gyroscope X) FFT data
        elseif id == '4'
            gyro_x_fft_data = read(s,L_imu/2,"single");
            gyro_x_fft_plot.Children.YData = 10*log10(gyro_x_fft_data);
            
        % IMU (Gyroscope Y) FFT data    
        elseif id == '5'
            gyro_y_fft_data = read(s,L_imu/2,"single");
            gyro_y_fft_plot.Children.YData = 10*log10(gyro_y_fft_data);
            
        % IMU (Gyroscope Z) FFT data    
        elseif id == '6'
            gyro_z_fft_data = read(s,L_imu/2,"single");
            gyro_z_fft_plot.Children.YData = 10*log10(gyro_z_fft_data);
        end 
    end
    
    catch ME
        % Catching the figure close event
        if strcmp(ME.identifier, 'MATLAB:class:InvalidHandle')
            pause(0.01);
            flush(s);
            pause(0.01);
            write(s, 's', "char");
            flush(s);
            clear s;
        else
            rethrow(ME);
        end
end
