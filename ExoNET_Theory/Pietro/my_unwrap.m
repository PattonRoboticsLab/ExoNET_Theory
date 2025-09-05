function unwrapped = my_unwrap(angles, tol)
    unwrapped = angles;
    for i = 2:length(angles)
        delta = unwrapped(i) - unwrapped(i-1);
        if delta > tol
            unwrapped(i:end) = unwrapped(i:end) - 2*pi;
        elseif delta < -tol
            unwrapped(i:end) = unwrapped(i:end) + 2*pi;
        end
    end
end