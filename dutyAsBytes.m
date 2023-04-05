function byte = dutyAsBytes(duty)
    % range of a 16 bit signed short used for duty cycle
    % MSB (sign) is cooling or heating mode

    % duty cycle will be < 0 if cooling needed Hi, INV
    % send msb = Hi, bits = INV
    % duty cycle will be > 0 if heating needed, Low, PWM
    % send msb = Low, bits = PWM
    
    byte = round(duty*127);
    
    if byte < 0
        byte = byte + 256;
    end
    byte = char(byte);
end

