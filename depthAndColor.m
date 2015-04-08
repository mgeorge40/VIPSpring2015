function analyzeHSV(imageName)
getPoints(imageName)
disp('Click 5 points on the color sample you want to analyze (suggested: four corners and the middle).')
end

function points = getPoints(imageName)

clc
close all
figure('WindowButtonDownFcn',@wbdcb)
RGB = imread(imageName);
image(RGB);
axis image;
axis off;
h = imgca;
points = [];
click = 0;
    function wbdcb(src,callbackdata)
    if strcmp(get(src,'SelectionType'),'normal')
        cp = get(h,'CurrentPoint');
        x = cp(1,1); %these give you the x and y coordinates that it draws the point at
        y = cp(1,2);
        x = floor(x);
        y = floor(y);
        click = click+1
        points(1, click) = x;
        points(2, click) = y;
        if click == 5
            makeRed(imageName, points)
        end
    end
end
end

function makeRed(imageName, points)
Hvalues = 0;
Svalues = 0;
Vvalues = 0;
RGB = imread(imageName);
HSV = rgb2hsv(RGB);
hue = HSV(:,:,1); 
sat = HSV(:,:,2); 
val = HSV(:,:,3);
for num = 1:5
    Hvalues = Hvalues + HSV(points(2, num), points(1, num), 1);
    Svalues = Svalues + HSV(points(2, num), points(1, num), 2);
    Vvalues = Vvalues + HSV(points(2, num), points(1, num), 3);
end
Haverage = Hvalues/5;
Saverage = Svalues/5;
Vaverage = Vvalues/5;
tolH = input('Hue tolerance?\n');
tolS = input('Saturation tolerance?\n');
tolV = input('Value tolerance?\n');
maskH = hue <= (Haverage + tolH) & hue >= (Haverage - tolH);
maskS = sat <= (Saverage + tolS) & sat >= (Saverage - tolS);
maskV = val <= (Vaverage + tolV) & val >= (Vaverage - tolV);
% redPoint = RGB(points(2,1), points(1,1), 1);
% greenPoint = RGB(points(2,1), points(1,1), 2);
% bluePoint = RGB(points(2,1), points(1,1), 3);
% maskR = red <= (redPoint + tol) & red >= (redPoint - tol);
% maskG = green <= (greenPoint + tol) & green >= (greenPoint - tol);
% maskB = blue <= (bluePoint + tol) & blue >= (bluePoint - tol);
hue(maskH & maskS & maskV) = 1;
sat(maskH & maskS & maskV) = 1;
val(maskH & maskS & maskV) = 1;
out = cat(3, hue, sat, val);
out = hsv2rgb(out);
imshow(out)

end