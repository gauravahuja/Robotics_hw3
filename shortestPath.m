function p = shortestPath(cx, cy, tx, ty, room, size)
    temp = ones(size, size)*10000;
    temp(ty, tx) = 0;
    filledStack = ones(1, 2)*-1;
    filledStack(1, 1) = tx;
    filledStack(1, 2) = ty;
    filledStackSize = 1;
    while (temp(cy, cx) == 10000) & (filledStackSize > 0)
        tix = filledStack(1,1);
        tiy = filledStack(1,2);
        newPathSize = temp(tiy, tix) +1;
        filledStack(1, :) = [];
        filledStackSize = filledStackSize-1;
        n = neighbors(tix, tiy, size);
        for i = [1:4]
            nix = n(i, 1);
            niy = n(i, 2);
            if (nix == -1 || niy == -1)
                continue;
            end
            if(room(niy, nix) == 125) %Obstacle
                continue;
            end
            if(temp(niy, nix) > newPathSize)
                temp(niy, nix) = newPathSize;
                filledStack = [filledStack; nix, niy];
                filledStackSize = filledStackSize+1;
            end
        end
    end
    tix = cx;
    tiy = cy;
    pathSize = temp(tiy, tix);
    p = [];
    fprintf('c = (%d, %d), t = (%d, %d)\n', cx, cy, tx, ty);
    while(pathSize > 0 & pathSize < 10000)
        n = neighbors(tix, tiy, size);
        for i = [1:4]
            nix = n(i, 1);
            niy = n(i, 2);
            if(nix > 0 && niy > 0)
                if(temp(niy, nix) == pathSize - 1)
                    tix = nix;
                    tiy = niy;
                    pathSize = pathSize - 1;
                    p = [p; tix, tiy];
                    break;
                end
            end
        end
    end
end