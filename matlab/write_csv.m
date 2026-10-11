function write_csv(file, header, M)
% WRITE_CSV  Write matrix M to a CSV file with a one-line header (works in MATLAB and GNU Octave).
fid = fopen(file, 'w'); fprintf(fid, '%s\n', header); fclose(fid);
dlmwrite(file, M, '-append', 'precision', 12);
end
