# Inisialisasi list data dengan 100 elemen
data = [i for i in range(100)]

# Inisialisasi buffer dengan 20 elemen pertama dari data
buffer_size = 20
buffer = [data[i:i+buffer_size] for i in range(len(data) - buffer_size + 1)]

# Mengakses buffer
for i in range(len(buffer)):
    print("Buffer[{}]: {}".format(i, buffer[i]))
