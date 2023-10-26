
function convertLasFile(lasfile, outfile)
% lasfile = '/media/jhuai/BackupPlus/jhuai/data/whu_tls/project2/20.las';
[folder, baseFileNameNoExt, extension] = fileparts(lasfile);
if nargin < 2
    outfile = fullfile(folder, [baseFileNameNoExt, '_a.ply']);
end
lasReader = lasFileReader(lasfile);
if hasCRSData(lasReader)
    crs = readCRS(lasReader);
%         disp(crs);
%         disp(crs.GeographicCRS);
%         disp(crs.GeographicCRS.Spheroid);
%         disp(crs.ProjectionParameters);
else
    disp("No CRS data available.");
end
[ptCloud,pointAttributes] = readPointCloud(lasReader,"Attributes","Classification");

% labels = label2rgb(pointAttributes.Classification); % this field is filled with zeros.
% gridStep = 0.1;
%     ptCloudA = pcdownsample(ptCloud,'gridAverage',gridStep);
% close all;
% figure;
% pcshow(ptCloud.Location, ptCloud.Color); hold on;

% fprintf('number of points %d\n', size(ptCloud.Intensity, 1));
%     format longg
%     ptCloud.Location(1, :)
%     ptCloud.Intensity(1)
%     ptCloud.Location(end, :)
%     ptCloud.Intensity(end)

% convert colors
if isa(ptCloud.Color(1, 1), 'uint16')
    % fprintf('Converting uint16 color to uint8...\n');
    c8 = uint8(ptCloud.Color);
    for i=1:3
        x = max(ptCloud.Color(:, i));
        n = min(ptCloud.Color(:, i));
        c8(:, i) = (double(ptCloud.Color(:, i)) - double(n)) * 255 / double(x - n);
    end
    cpc = pointCloud(ptCloud.Location, 'Color', c8);
    pcwrite(cpc, outfile, "Encoding", "ascii");
    fprintf("Saving %s to %s\n", lasfile, outfile);
else
    pcwrite(ptCloud, outfile, "Encoding", "ascii");
    fprintf("Saving %s to %s\n", lasfile, outfile);
end
