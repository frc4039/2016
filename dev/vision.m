function vision
    raw = imread('practice_field_blue.bmp');
    %colormap(map);
    L = 480;
    W = 640;
    H = 3;
        
% #define R_THRESHOLD 210
% #define G_THRESHOLD 190
% #define B_THRESHOLD 210
% 	inline void BinaryFilter(void){
% 		for (int i = 0; i < (RES_X*RES_Y); i++){
% 			/*
% 			int x = i % RES_X;
% 			int y = (int)(i / RES_X);
% 			if (y > 400)
% 				printf("R, G, B: (%d, %d, %d), (%d, %d)\n", R[i*4], G[i*4], B[i*4], x, y);
% 			*/
% 			//if it has lots of red
% 			if ((R[i*4] > R_THRESHOLD)){
% 				proc_pixel[i] = 0;
% 			}
% 			//blue without green
% 			else if (((B[i*4] > B_THRESHOLD) && (G[i*4] < G_THRESHOLD))){
% 				//printf("eliminating pixel BG: %d, %d\n", B[i*4], G[i*4]);
% 				proc_pixel[i] = 0;
% 			}
% 			//if lots of green
% 			else if ((G[i*4] > G_THRESHOLD)){
% 				//printf("got green pixel: %d\n", G[i*4]);
% 				proc_pixel[i] = 255;
% 			}
% 			else
% 				proc_pixel[i] = 0;
% 		}
% 	}
    %raw(1)
    %Binary Filter
    R = 100;
    G = 120;
    B = 100;
    
    bin = linspace(L, W);
    test = linspace(L, W);
    for i = 1:L
        for j = 1:W
            %put this part into the c++ code
            if(raw(i,j,1) > R) 
                pix = 0;
            elseif (raw(i, j, 2) < G && raw(i, j, 3) > B)
                pix = 0;
            elseif (raw(i, j, 2) > G)
                pix = 255;
            else
                pix = 0;
            end
            
            bin(i,j) = pix;
        end
    end
    %bin
   % image(bin);
    image(raw);
 