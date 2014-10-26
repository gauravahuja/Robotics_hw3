function valid = insideRoom(x, y, size)
    valid = 1;
    if (x < 1 || x > size || y < 1 || y > size)
        valid = 0;
    end
end