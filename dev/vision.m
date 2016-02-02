function vision
    raw = imread('RealFullField/3.jpg');
    %colormap(gray);
    L = 480;
    W = 640;
    H = 3;
        
    raw(1)
    %Binary Filter
    threshold =100;
    similarity = 10;
    bin = linspace(L, W);
    test = linspace(L,W);
    for i = 1:L
        for j = 1:W
            %put this part into the c++ code
            if(abs(raw(i,j,3)-raw(i,j,2)) > similarity) 
                pix = raw(i,j,2) - raw(i,j,3) - raw(i,j,1);
            else
                pix = raw(i,j,2) - raw(i,j,1);
            end
            
            if (pix > threshold)
                bin(i,j) = 255;
            else
                bin(i,j) = 0;
            end
            test(i,j) = raw(i,j,1);
        end
    end
    bin
    image(bin);
    %imagesc(test);
 