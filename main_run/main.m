% Kevin Shi (ks3942)

clear all;
close all;


% Read images
filename1 = "img1.png";
filename2 = "img2.png";

img1 = imread(filename1);
img2 = imread(filename2);

%figure(1); imshow(img1); title('img1');
%figure(2); imshow(img2); title('img2');


%     1 (10 points) Hard Coding Point Correspondences
img1_pts = [571, 66; 387, 156; 439, 404; 247, 496];
img2_pts = [375, 86; 185, 178; 231, 428; 44, 510];
imgs_combined = [img1, img2];

figure(1);
imshow(imgs_combined);
hold on;
offset = size(img1, 2);
colors = {'r', 'y', 'b', 'g'};
for i = 1:4
    plot(img1_pts(i, 1), img1_pts(i, 2), '.', 'MarkerSize', 10, 'LineWidth', 2, 'Color', colors{i});
    plot(img2_pts(i, 1) + offset, img2_pts(i, 2), '.', 'MarkerSize', 10, 'LineWidth', 2, 'Color', colors{i});
end
title('Manual Correspondences');


%     2 (30 points) Compute Transformation Matrix, Project, and Blend!
% Compute homography matrix (assumes img1 is the base image)
A = zeros(8, 9);
B = zeros(8, 9);

for i = 1:4
    x  = img1_pts(i, 1);
    y  = img1_pts(i, 2);
    xp = img2_pts(i, 1);
    yp = img2_pts(i, 2);

    A(2*i-1, :) = [x, y, 1, 0, 0, 0, 0, 0, 0];
    A(2*i, :)   = [0, 0, 0, x, y, 1, 0, 0, 0];

    B(2*i-1, :) = [0, 0, 0, 0, 0, 0, xp*x, xp*y, xp];
    B(2*i, :)   = [0, 0, 0, 0, 0, 0, yp*x, yp*y, yp];
end

[vec,val] = eig(A'*A + B'*B - B'*A - A'*B)

[~, idx] = min(diag(val))
m = vec(:, idx)
H = reshape(m, [3, 3])'
A
B

% Determine the dimensions of the new combined image
[h_img1, w_img1, c_img1] = size(img1);
[h_img2, w_img2, c_img2] = size(img2);

% Follows the last slide in 2D Transformations to copy the transformed content into a new image.
% To determine the new dimensions,
% Apply the transformation matrix to each corner of your image, 
% getting the min and max of the x and y coordinates.
img1_corners = [1 w_img1 1 w_img1;
                1 1 h_img1 h_img1;
                1 1 1 1];
img1_corners_transformed = H*img1_corners
% we can just divide by the homogenous coordinate (perspective division) to return to scale!
img1_corners_transformed = img1_corners_transformed ./ img1_corners_transformed(3, :)
x_coordinates_img1 = img1_corners_transformed(1, :)
y_coordinates_img1 = img1_corners_transformed(2, :)

img1_min_X = floor(min(x_coordinates_img1))
img1_max_X = ceil(max(x_coordinates_img1))
img1_min_Y = floor(min(y_coordinates_img1))
img1_max_Y = ceil(max(y_coordinates_img1))

% The height of this image should be the maximum of the base image’s height or the maximum projected y value from the other image. 
% The width will be equal to the maximum of the base image’s width or the maximum projected x value from the other image.
canvas_offset = [1 - img1_min_X, 1 - img1_min_Y]
new_img_h = max(h_img1, img1_max_Y) + canvas_offset(2)
new_img_w = max(w_img1, img1_max_X) + canvas_offset(1)
new_img = zeros(new_img_h, new_img_w, 3, 'uint8');

% Finally we need to populate our new image with pixel(s) from the base and projected images.
inverse_H = inv(H);
for x_new = 1:new_img_w
    for y_new = 1:new_img_h
        % Add the mins to this location and apply the inverse transformation matrix to that.
        % This provide the location in the original image.
        location = [x_new - canvas_offset(1); y_new - canvas_offset(2); 1];
        %location = [x_new + img1_min_X; y_new + img1_min_Y; 1];
        original_img_location = inverse_H * location;
        original_img_location = original_img_location ./ original_img_location(3);
        x_original_location = original_img_location(1);
        y_original_location = original_img_location(2);

        % If the location is within bounds of the original image, 
        % put the content from the original image at this computed location 
        % (nearest neighbor or interpolated) into the new image at the current location.
        x_nn = round(x_original_location);
        y_nn = round(y_original_location);

        if x_nn >= 1 && x_nn <= w_img1 && y_nn >= 1 && y_nn <= h_img1
            new_img(y_new, x_new, :) = img1(y_nn, x_nn, :);
        end
    end
end

% If both images map to that location, you’ll want to blend them
[h_new_img, w_new_img, c_new_img] = size(new_img);
store_new_img = new_img;
for y = 1:h_img2
    for x = 1:w_img2
        y_off = y + canvas_offset(2);
        x_off = x + canvas_offset(1);
        if x_off >= 1 && x_off <= w_new_img && y_off >= 1 && y_off <= h_new_img
            if all(new_img(y_off, x_off, :) > 0)
                store_new_img(y_off, x_off, :) = 0.5 * double(store_new_img(y_off, x_off, :)) + 0.5 * double(img2(y, x, :));
            else
                store_new_img(y_off, x_off, :) = img2(y, x, :);
            end
        end
    end
end

figure(2); imshow(store_new_img); title('Stitched images using manual correspondences');


%     3 (10 points) Keypoint Extraction
sigma = 2;
img1_gray = im2double(rgb2gray(img1));
img2_gray = im2double(rgb2gray(img2));

img1_smoothed = imgaussfilt(img1_gray, sigma);
img2_smoothed = imgaussfilt(img2_gray, sigma);

% DoG image
img1_dog = img1_gray - img1_smoothed;
img2_dog = img2_gray - img2_smoothed;

[img1_Gx, img1_Gy] = imgradientxy(img1_dog, 'central');
img1_grad_mag = sqrt(img1_Gx.^2 + img1_Gy.^2);
[img2_Gx, img2_Gy] = imgradientxy(img2_dog, 'central');
img2_grad_mag = sqrt(img2_Gx.^2 + img2_Gy.^2);

% Find local maxima in a 3x3 neighborhood for img1
img1_grad_mag
all_keypoints_img1 = [];
contrast_thresh_img1 = 0.05;
[rows_img1, cols_img1] = size(img1_dog);
for i = 2:rows_img1-1
    for j = 2:cols_img1-1
        neighborhood = img1_dog(i-1:i+1, j-1:j+1);
        center = neighborhood(2, 2);
        if center == max(neighborhood(:)) && abs(center) > contrast_thresh_img1
            all_keypoints_img1(end+1, :) = [j, i];
        end
    end
end
figure(3);
imshow(img1);
hold on;
for i = 1:size(all_keypoints_img1, 1)
    plot(all_keypoints_img1(i, 1), all_keypoints_img1(i, 2), 'o', 'MarkerSize', 3, 'LineWidth', 1, 'Color', 'r');
end
title('All extrema points in image 1');
hold off;
% Prune keypoints for img1 (Remove keypoints that are in areas of low contrast and near borders)
pruned_keypoints_img1 = [];
border = 5;
bottom_border = 0.1*rows_img1;
for k = 1:size(all_keypoints_img1, 1)
    x = all_keypoints_img1(k, 1);
    y = all_keypoints_img1(k, 2);
    if x > border && x < cols_img1 - border && y > border && y < rows_img1 - bottom_border
        if img1_grad_mag(y, x) > contrast_thresh_img1
            pruned_keypoints_img1(end+1, :) = [x, y];
        end
    end
end
figure(4);
imshow(img1);
hold on;
for i = 1:size(pruned_keypoints_img1, 1)
    plot(pruned_keypoints_img1(i, 1), pruned_keypoints_img1(i, 2), 'o', 'MarkerSize', 3, 'LineWidth', 1, 'Color', 'r');
end
title('Pruned extrema points in image 1');
hold off;

% Find local maxima in a 3x3 neighborhood for img2
img2_grad_mag
all_keypoints_img2 = [];
contrast_thresh_img2 = 0.05;
[rows_img2, cols_img2] = size(img2_dog);
for i = 2:rows_img2-1
    for j = 2:cols_img2-1
        neighborhood = img2_dog(i-1:i+1, j-1:j+1);
        center = neighborhood(2, 2);
        if center == max(neighborhood(:)) && abs(center) > contrast_thresh_img2
            all_keypoints_img2(end+1, :) = [j, i];
        end
    end
end
figure(5);
imshow(img2);
hold on;
for i = 1:size(all_keypoints_img2, 1)
    plot(all_keypoints_img2(i, 1), all_keypoints_img2(i, 2), 'o', 'MarkerSize', 3, 'LineWidth', 1, 'Color', 'r');
end
title('All extrema points in image 2');
hold off;
% Prune keypoints for img2 (Remove keypoints that are in areas of low contrast and near borders)
pruned_keypoints_img2 = [];
border = 5;
bottom_border = 0.1*rows_img2;
for k = 1:size(all_keypoints_img2, 1)
    x = all_keypoints_img2(k, 1);
    y = all_keypoints_img2(k, 2);
    if x > border && x < cols_img2 - border && y > border && y < rows_img2 - bottom_border
        if img2_grad_mag(y, x) > contrast_thresh_img2
            pruned_keypoints_img2(end+1, :) = [x, y];
        end
    end
end
figure(6);
imshow(img2);
hold on;
for i = 1:size(pruned_keypoints_img2, 1)
    plot(pruned_keypoints_img2(i, 1), pruned_keypoints_img2(i, 2), 'o', 'MarkerSize', 3, 'LineWidth', 1, 'Color', 'r');
end
title('Pruned extrema points in image 2');
hold off;


%     4 (10 points) Keypoint Description and Matching
% 4.1 Descriptors
patch_size = 9;
img1_double = im2double(img1);
img2_double = im2double(img2);
[h_img1, w_img1, c_img1] = size(img1);
[h_img2, w_img2, c_img2] = size(img2);

patch_halved = floor(patch_size / 2);
descriptors_img1 = [];
descriptors_img2 = [];

for i = 1:size(pruned_keypoints_img1, 1)
    x = pruned_keypoints_img1(i, 1);
    y = pruned_keypoints_img1(i, 2);

    % Check if the patch fits within the image bounds
    if x > patch_halved && x <= w_img1 - patch_halved && y > patch_halved && y <= h_img1 - patch_halved
        patch = img1(y-patch_halved:y+patch_halved, x-patch_halved:x+patch_halved, :);
        descriptor = patch(:)';  %flatten
        descriptors_img1 = [descriptors_img1; descriptor];
    end
end
for i = 1:size(pruned_keypoints_img2, 1)
    x = pruned_keypoints_img2(i, 1);
    y = pruned_keypoints_img2(i, 2);

    % Check if the patch fits within the image bounds
    if x > patch_halved && x <= w_img2 - patch_halved && y > patch_halved && y <= h_img2 - patch_halved
        patch = img2(y-patch_halved:y+patch_halved, x-patch_halved:x+patch_halved, :);
        descriptor = patch(:)';  %flatten
        descriptors_img2 = [descriptors_img2; descriptor];
    end
end

% 4.2 Keypoint Correspondences
%step 1
C_1 = [];
thresh_y = 50; % Correspondences should have roughly the same y value.
for i = 1:size(pruned_keypoints_img1, 1)
    y1 = pruned_keypoints_img1(i, 2);
    d1 = descriptors_img1(i, :);

    % Find candidates in pruned_keypoints_img2 with similar y-value
    candidates = abs(pruned_keypoints_img2(:,2) - y1) < thresh_y;
    kp2_cands = pruned_keypoints_img2(candidates, :);
    desc2_cands = descriptors_img2(candidates, :);

    if isempty(kp2_cands)
        continue;
    end
    
    % Find best match using Euclidean distance
    distances = sqrt(sum((desc2_cands - d1).^2, 2));
    [~, min_idx] = min(distances);

    % Map local index back to actual
    real_idx = find(candidates);
    C_1 = [C_1; i, real_idx(min_idx)];
end

%step 2
C_2 = [];
for i = 1:size(pruned_keypoints_img2, 1)
    y2 = pruned_keypoints_img2(i, 2);
    d2 = descriptors_img2(i, :);

    % Find candidates in pruned_keypoints_img1 with similar y-value
    candidates = abs(pruned_keypoints_img1(:,2) - y2) < thresh_y;
    kp1_cands = pruned_keypoints_img1(candidates, :);
    desc1_cands = descriptors_img1(candidates, :);

    if isempty(kp1_cands)
        continue;
    end

    % Find best match using Euclidean distance
    distances = sqrt(sum((desc1_cands - d2).^2, 2));
    [~, min_idx] = min(distances);

    % Map local index back to actual
    real_idx = find(candidates);
    C_2 = [C_2; real_idx(min_idx), i];
end

%step 3
[~, first, second] = intersect(C_1, C_2, 'rows');
C = C_1(first, :);

%step 4
C_final = [];
dist_thresh = 1000;
for i = 1:size(C, 1)
    d1 = double(descriptors_img1(C(i, 1), :));
    d2 = double(descriptors_img2(C(i, 2), :));
    dist = norm(d1 - d2)

    if dist < dist_thresh
        C_final = [C_final; C(i, :)];
    end
end

C_final
C

num_to_show = min(size(C_final, 1), 10);
combined = [img1, img2];
offset = size(img1, 2);
figure(7);
imshow(combined);
hold on;
for i = 1:num_to_show
    pt1 = pruned_keypoints_img1(C_final(i, 1), :);
    pt2 = pruned_keypoints_img2(C_final(i, 2), :) + [offset, 0];

    plot([pt1(1), pt2(1)], [pt1(2), pt2(2)], 'r-');
    plot(pt1(1), pt1(2), 'r.');
    plot(pt2(1), pt2(2), 'r.');
end
title('Some Point Correspondences');
hold off;


%     5 (10 points) Find the Transformation Matrix via RANSAC and Stitch
num_in_C_final = size(C_final, 1);
best_pc = [];
min_distance_threshold = 20;

% 1. For experiments 1 through N (you choose N)
for i = 1:20000
    % (a) Select four correspondences at random.
    idx = randperm(num_in_C_final, 4);
    sample = C_final(idx, :);

    source = pruned_keypoints_img1(sample(:,1), :);
    destination = pruned_keypoints_img2(sample(:,2), :);

    too_close = false;
    for j = 1:3
        for k = j+1:4
            d_x = source(j,1) - source(k,1);
            d_y = source(j,2) - source(k,2);
            dist_sq = d_x^2 + d_y^2;
            if dist_sq < min_distance_threshold^2
                too_close = true;
                break;
            end
        end
        if too_close
            break;
        end
    end
    if too_close
        continue;
    end

    % (b) Compute the transformation matrix using these correspondences.
    A = zeros(8, 9);
    B = zeros(8, 9);
    
    for i = 1:4
        x  = source(i, 1);
        y  = source(i, 2);
        xp = destination(i, 1);
        yp = destination(i, 2);
    
        A(2*i-1, :) = [x, y, 1, 0, 0, 0, 0, 0, 0];
        A(2*i, :)   = [0, 0, 0, x, y, 1, 0, 0, 0];
    
        B(2*i-1, :) = [0, 0, 0, 0, 0, 0, xp*x, xp*y, xp];
        B(2*i, :)   = [0, 0, 0, 0, 0, 0, yp*x, yp*y, yp];
    end
    
    [vec,val] = eig(A'*A + B'*B - B'*A - A'*B);
    [~, idx] = min(diag(val));
    m = vec(:, idx);
    H = reshape(m, [3, 3])'

    % (c) Using the discovered transformation matrix, count how many point 
    % correspondences (among all of them) would end up within a few pixels of one another after projection.
    n = size(pruned_keypoints_img1(C_final(:,1), :),1);
    pts_h = [pruned_keypoints_img1(C_final(:,1), :), ones(n,1)]';
    proj = H * pts_h;
    proj = proj ./ proj(3,:);
    pts_proj = proj(1:2, :)';

    kp2_actual = pruned_keypoints_img2(C_final(:,2), :);
    dists = sqrt(sum((pts_proj - kp2_actual).^2, 2));

    % 2. Keep the transformation matrix the resulting in the largest number of point 
    % correspondences (among all of them) that ended up within a few pixels of one another after projection.
    keep_idx = find(dists < 4);
    if length(keep_idx) > length(best_pc)
        best_pc = keep_idx;
        best_H = H;
        best_sample = sample;
    end
end
best_sample

% Draw lines between the keypoint coorespondences used to computer your final transformation matrix.
combined = [img1, img2];
offset = size(img1, 2);
figure(8);
imshow(combined);
hold on;
for i = 1:4
    pt1 = pruned_keypoints_img1(best_sample(i, 1), :);
    pt2 = pruned_keypoints_img2(best_sample(i, 2), :) + [offset, 0];

    plot([pt1(1), pt2(1)], [pt1(2), pt2(2)], 'r-');
    plot(pt1(1), pt1(2), 'r.');
    plot(pt2(1), pt2(2), 'r.');
end
title('Point Correspondences for final transformation matrix');
hold off;

% Now use this transformation matrix to stitch your images!
A = zeros(8, 9);
B = zeros(8, 9);

for i = 1:4
    x  = pruned_keypoints_img1(best_sample(i, 1), 1);
    y  = pruned_keypoints_img1(best_sample(i, 1), 2);
    xp = pruned_keypoints_img2(best_sample(i, 2), 1);
    yp = pruned_keypoints_img2(best_sample(i, 2), 2);

    A(2*i-1, :) = [x, y, 1, 0, 0, 0, 0, 0, 0];
    A(2*i, :)   = [0, 0, 0, x, y, 1, 0, 0, 0];

    B(2*i-1, :) = [0, 0, 0, 0, 0, 0, xp*x, xp*y, xp];
    B(2*i, :)   = [0, 0, 0, 0, 0, 0, yp*x, yp*y, yp];
end

[vec,val] = eig(A'*A + B'*B - B'*A - A'*B);
[~, idx] = min(diag(val));
m = vec(:, idx);
H = reshape(m, [3, 3])'


% Determine the dimensions of the new combined image
[h_img1, w_img1, c_img1] = size(img1);
[h_img2, w_img2, c_img2] = size(img2);

% Follows the last slide in 2D Transformations to copy the transformed content into a new image.
% To determine the new dimensions,
% Apply the transformation matrix to each corner of your image, 
% getting the min and max of the x and y coordinates.
img1_corners = [1 w_img1 1 w_img1;
                1 1 h_img1 h_img1;
                1 1 1 1];
img1_corners_transformed = H*img1_corners
% we can just divide by the homogenous coordinate (perspective division) to return to scale!
img1_corners_transformed = img1_corners_transformed ./ img1_corners_transformed(3, :)
x_coordinates_img1 = img1_corners_transformed(1, :)
y_coordinates_img1 = img1_corners_transformed(2, :)

img1_min_X = floor(min(x_coordinates_img1))
img1_max_X = ceil(max(x_coordinates_img1))
img1_min_Y = floor(min(y_coordinates_img1))
img1_max_Y = ceil(max(y_coordinates_img1))

% The height of this image should be the maximum of the base image’s height or the maximum projected y value from the other image. 
% The width will be equal to the maximum of the base image’s width or the maximum projected x value from the other image.
canvas_offset = [1 - img1_min_X, 1 - img1_min_Y]
new_img_h = max(h_img1, img1_max_Y) + canvas_offset(2)
new_img_w = max(w_img1, img1_max_X) + canvas_offset(1)
new_img = zeros(new_img_h, new_img_w, 3, 'uint8');

% Finally we need to populate our new image with pixel(s) from the base and projected images.
inverse_H = inv(H);
for x_new = 1:new_img_w
    for y_new = 1:new_img_h
        % Add the mins to this location and apply the inverse transformation matrix to that.
        % This provide the location in the original image.
        location = [x_new - canvas_offset(1); y_new - canvas_offset(2); 1];
        %location = [x_new + img1_min_X; y_new + img1_min_Y; 1];
        original_img_location = inverse_H * location;
        original_img_location = original_img_location ./ original_img_location(3);
        x_original_location = original_img_location(1);
        y_original_location = original_img_location(2);

        % If the location is within bounds of the original image, 
        % put the content from the original image at this computed location 
        % (nearest neighbor or interpolated) into the new image at the current location.
        x_nn = round(x_original_location);
        y_nn = round(y_original_location);

        if x_nn >= 1 && x_nn <= w_img1 && y_nn >= 1 && y_nn <= h_img1
            new_img(y_new, x_new, :) = img1(y_nn, x_nn, :);
        end
    end
end

% If both images map to that location, you’ll want to blend them
[h_new_img, w_new_img, c_new_img] = size(new_img);
store_new_img = new_img;
for y = 1:h_img2
    for x = 1:w_img2
        y_off = y + canvas_offset(2);
        x_off = x + canvas_offset(1);
        if x_off >= 1 && x_off <= w_new_img && y_off >= 1 && y_off <= h_new_img
            if all(new_img(y_off, x_off, :) > 0)
                store_new_img(y_off, x_off, :) = 0.5 * double(store_new_img(y_off, x_off, :)) + 0.5 * double(img2(y, x, :));
            else
                store_new_img(y_off, x_off, :) = img2(y, x, :);
            end
        end
    end
end

figure(9); imshow(store_new_img); title('Automated final stitched image');
