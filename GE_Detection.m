function [HeelStrike, ToeOff, Z_Filter, timestamps_new] = GE_Detection(timestamps, gyroZ, Fs_new, f_low)
% GE_Detection  对陀螺仪 Z 轴信号进行重采样、滤波并检测脚跟落地/脚尖离地时刻
%
%   [HeelStrike, ToeOff, Z_Filter, timestamps_new] = GE_Detection(timestamps, gyroZ)
%   [HeelStrike, ToeOff, Z_Filter, timestamps_new] = GE_Detection(timestamps, gyroZ, Fs_new, f_low)

    if nargin < 3 || isempty(Fs_new)
        Fs_new = 100;
    end
    if nargin < 4 || isempty(f_low)
        f_low = 7;
    end

    t_start = min(timestamps(:));
    t_end   = max(timestamps(:));
    timestamps_new = t_start : 1/Fs_new : t_end;
    signal_resampled = interp1(timestamps, gyroZ, timestamps_new, 'linear');

    % 2-order Butterworth filtering
    [B, A] = butter(2, 2*f_low/Fs_new, 'low');
    Z_Filter = filtfilt(B, A, signal_resampled);


    Omega_Z = -Z_Filter;           
    ThresholdSwitch = false;
    HeelStrikeCount   = 0;
    ThresholdSwitchTime = [];
    HeelStrike = [];
    ToeOff     = [];

    % GE Detection
    for i = 2 : length(Omega_Z)
        if ~ThresholdSwitch && Omega_Z(i-1) >= 180 && Omega_Z(i) < 180
            ThresholdSwitch = true;
            HeelStrikeCount = HeelStrikeCount + 1;
            ThresholdSwitchTime(HeelStrikeCount) = i;
        end
        if ThresholdSwitch
            if Omega_Z(i-1) <= 0 && Omega_Z(i) > Omega_Z(i-1)
                HeelStrike(HeelStrikeCount) = i - 1;
                for ii = ThresholdSwitchTime(HeelStrikeCount)-1 : -1 : 2
                    if Omega_Z(ii) <= 0 && Omega_Z(ii) < Omega_Z(ii-1)
                        ToeOff(HeelStrikeCount) = ii;
                        break;
                    end
                end
                ThresholdSwitch = false;
            end
        end
    end
end