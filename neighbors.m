function n = neighbors(x, y, size)
    xn = [x+1, x-1,x, x];
    yn = [y, y, y+1, y-1];
    n = ones(4, 2)*-1;
    for i = 1:4
        if insideRoom(xn(i), yn(i), size) == 1
            n(i, 1) = xn(i);
            n(i, 2) = yn(i);
        end
    end
end
