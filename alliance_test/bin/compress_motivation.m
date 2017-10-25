function compressed_data = compress_motivation(data)

  data_size = rows(data);
  k = 1; p = k;
  compressed_data(k, :) = data(1, :);
  for i = 2 : data_size
    % search for derivative changes, instead
    if all(eq(compressed_data(k, 2 : end), data(i - 1, 2 : end))) && ~all(eq(compressed_data(k, 2 : end), data(i, 2 : end)))
      if ne(i - 1, p)
        k = k + 1;
        compressed_data(k, :) = data(i - 1, :);
      end;
      k = k + 1;
      compressed_data(k, :) = data(i, :);
      p = i;
    end;
  end;
  if ne(i, p)
    k = k + 1;
    compressed_data(k, :) = data(i - 1, :);
  end;
  compressed_data_size = rows(compressed_data);
  disp(['Compression rate: ' num2str((data_size - compressed_data_size) / data_size * 100) ' [%]']);

end
