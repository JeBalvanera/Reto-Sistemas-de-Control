clearvars;
close all;
clc;

% Configuración del puerto serial
port = "COM9"; 
baudRate = 115200;
pserial = serialport(port, baudRate);
pserial.Timeout = 2; 
configureTerminator(pserial, "CR/LF"); 
flush(pserial); 

disp('Esperando a que Arduino esté listo...');
pause(2); 

% Reservar memoria
NumSamples = 300;
T = zeros(1, NumSamples);
R = zeros(1, NumSamples); 
Y = zeros(1, NumSamples); 

disp('Comenzando prueba del PID...');
tic;
for k = 1:NumSamples
    t = toc; 
    T(k) = t;
    if t > 1
        R(k) = 1.5; % Velocidad deseada en rev/s 
    else
        R(k) = 0.0;
    end
    writeline(pserial, num2str(R(k))); 
    strResponse = readline(pserial); 
    Y(k) = str2double(strResponse);
end

% Apagar el motor al terminar
writeline(pserial, "0");
clear pserial;
disp('Prueba finalizada.');

% --- Gráfica 1: Respuesta Experimental Arduino ---
figure('Name','Respuesta PID - Arduino','NumberTitle','off');
plot(T, R, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Referencia');
hold on;
plot(T, Y, 'r', 'LineWidth', 1.5, 'DisplayName', 'Velocidad Real');
legend('show', 'Location', 'southeast'); 
xlabel('Tiempo (s)');
ylabel('Velocidad (rev/s)');
grid on;

% --- Gráfica 2: Simulación Teórica con stepinfo_dos ---
Gs = tf(2.84, [0.105 1], 'InputDelay', 0.027);
Kp = 0.8215;
Ki = 7.8238;
Kd = 0.0110;

% Ejecuta la función del archivo externo
info = stepinfo_dos(Gs, Kp, Ki, Kd);

% Recortar datos a partir del escalón (t >= 1.0 s)
idx = T >= 1.0;
t_exp = T(idx) - 1.0; % Desplazar tiempo a t=0
y_exp = Y(idx);

% Calcular métricas del motor
info_real = stepinfo(y_exp, t_exp, 'SettlingTimeThreshold', 0.02);
disp('--- Métricas Reales del Motor ---');
disp(info_real);