import matplotlib.pyplot as plt

# Data untuk plot pertama
x1 = [1, 2, 3, 4, 5]
y1 = [2, 3, 5, 7, 11]

# Data untuk plot kedua
x2 = [1, 2, 3, 4, 5]
y2 = [1, 4, 9, 16, 25]

# Membuat subplot pertama
plt.subplot(2, 1, 1)  # 2 baris, 1 kolom, subplot pertama
plt.plot(x1, y1, color='blue', linestyle='-', marker='o', label='Plot 1')
plt.xlabel('Nilai X')
plt.ylabel('Nilai Y')
plt.title('Plot 1')
plt.legend()

# Membuat subplot kedua
plt.subplot(2, 1, 2)  # 2 baris, 1 kolom, subplot kedua
plt.plot(x2, y2, color='red', linestyle='--', marker='x', label='Plot 2')
plt.xlabel('Nilai X')
plt.ylabel('Nilai Y')
plt.title('Plot 2')
plt.legend()

# Menyesuaikan layout
plt.tight_layout()

# Menampilkan grafik
plt.show()


#  BUAT 6 PLOT DENGAN SUSUNAN 3 BARIS DAN 2 KOLOM
# Plot 1
plt.subplot(3, 2, 1)
plt.title('Plot 1')
plt.plot([1, 2, 3], [4, 5, 6])

# Plot 2
plt.subplot(3, 2, 2)
plt.title('Plot 2')
plt.plot([1, 2, 3], [7, 8, 9])

# Plot 3
plt.subplot(3, 2, 3)
plt.title('Plot 3')
plt.plot([1, 2, 3], [10, 11, 12])

# Plot 4
plt.subplot(3, 2, 4)
plt.title('Plot 4')
plt.plot([1, 2, 3], [13, 14, 15])

# Plot 5
plt.subplot(3, 2, 5)
plt.title('Plot 5')
plt.plot([1, 2, 3], [16, 17, 18])

# Plot 6
plt.subplot(3, 2, 6)
plt.title('Plot 6')
plt.plot([1, 2, 3], [19, 20, 21])

plt.tight_layout()  # Membuat tata letak subplot menjadi lebih rapi

plt.show()
