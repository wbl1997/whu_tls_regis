% Align TLS point clouds
% The pcregistericp in matlab 2021a is better than matlab 2022a and 2023a.
close all;

datadir = '/media/jhuai/BackupPlus/jhuai/data/whu_tls/project2';

seqpairs = [[3, 22], [21, 22]];

v = ver('MATLAB');
for p = 1:size(seqpairs, 1)
    s = seqpairs(p, :);
    basefile = [datadir, '/', num2str(s(1)), '.las'];
    basereader = lasFileReader(basefile);
    [fixed,pointAttributes] = readPointCloud(basereader,"Attributes","Classification");
    maxNumPoints = 12;
    outputlog = [datadir, '/', num2str(s(1)), '_', num2str(s(2)), '_icp.log'];
    logid = fopen(outputlog, 'w');
    fixedDownsampled = pcdownsample(fixed,"nonuniformGridSample",maxNumPoints);
    fprintf(logid, 'fixed points %d after downsample %d\n', size(fixed.Location, 1), size(fixedDownsampled.Location, 1));


    queryfile = [datadir, '/', num2str(s(2)), '.las'];
    if ~isfile(queryfile)
        continue;
    end
    
    fprintf('Registering %s to %s\n', queryfile, basefile);
    fprintf(logid, 'Registering %s to %s\n', queryfile, basefile);
    queryreader = lasFileReader(queryfile);
    [moving,pointAttributes] = readPointCloud(queryreader,"Attributes","Classification");
    movingDownsampled = pcdownsample(moving,"nonuniformGridSample",maxNumPoints);

    fprintf(logid, 'moving points %d after downsample %d\n', size(moving.Location, 1), size(movingDownsampled.Location, 1));
    [tform, movingAligned, rmse] = pcregistericp(movingDownsampled,fixedDownsampled, ...
        'Metric','pointToPoint','Extrapolate',true, 'InlierRatio', 0.9);
    ptCloudAligned = pctransform(moving,tform);
    figure;
    pcshowpair(ptCloudAligned, fixed);
    title([num2str(s(2)), ' to ', num2str(s(1))], 'Interpreter', 'none');

    if v.Release=="(R2023a)"
        fix_T_moving = tform.A;
    elseif v.Release=="(R2022a)"
        fix_T_moving = [tform.Rotation', tform.Translation'];
    elseif v.Release=="(R2021a)"
        fix_T_moving = [tform.Rotation, tform.Translation'];
    else
        fprintf('Unrecognized matlab release %s. Fall back to R2021a.\n', v.Release);
        fix_T_moving = [tform.Rotation, tform.Translation'];
    end

    fprintf(logid, 'Rmse %.3f, pose of the moving point cloud is:\n', rmse);
    fprintf(logid, '%.9f %.9f %.9f %.9f\n', fix_T_moving');
    
    outputfile = [datadir, '/W', num2str(s(1)), '_T_W', num2str(s(2)), '.txt'];
    write_transform(fix_T_moving, outputfile);

    fclose(logid);
end
