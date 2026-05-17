function make_gif()
% Build ilc_convergence.gif from ilc_iter_01.png .. ilc_iter_10.png.
    files = arrayfun(@(k) sprintf('ilc_iter_%02d.png', k), 1:10, ...
                     'UniformOutput', false);
    out = 'ilc_convergence.gif';
    for k = 1:numel(files)
        [A, map] = rgb2ind(imread(files{k}), 256);
        if k == 1
            imwrite(A, map, out, 'gif', 'LoopCount', Inf, 'DelayTime', 0.3);
        else
            imwrite(A, map, out, 'gif', 'WriteMode', 'append', 'DelayTime', 0.3);
        end
    end
    fprintf('Wrote %s (%d frames)\n', out, numel(files));
end
