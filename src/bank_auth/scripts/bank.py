#!/usr/bin/env python3
import rospy
import std_msgs
from std_msgs.msg import Bool, String

arduino_pub = rospy.Publisher('led_status', std_msgs.msg.Int32, queue_size=10)

saldo_pengguna = 1000000.0 
login_status = False 

respon_login = rospy.Publisher('respon_login', String, queue_size=10)
respon_transaksi = rospy.Publisher('respon_transaksi', String, queue_size=10)


def proses_login(data):
    global login_status
    rospy.loginfo("Menerima data deteksi login...")
    if data.data: 
        login_status = True
        rospy.loginfo("Login berhasil!")
        respon_login.publish("Login berhasil")
        tampilkan_menu()
    else:
        rospy.logwarn("Login gagal: Persegi panjang hitam tidak terdeteksi")
        respon_login.publish("Login gagal: Deteksi tidak valid")

def tampilkan_menu():
    global login_status, saldo_pengguna

    if not login_status:
        rospy.logwarn("Anda harus login terlebih dahulu.")
        return

    while login_status:
        rospy.loginfo("\n=== Layanan Bank ===")
        rospy.loginfo("1. Setoran")
        rospy.loginfo("2. Penarikan")
        rospy.loginfo("3. Cek Saldo")
        rospy.loginfo("4. Logout")
        rospy.loginfo("Silakan pilih layanan dengan mengetikkan angka (1-4):")

        try:
            pilihan = input("Masukkan pilihan Anda: ").strip()
            if pilihan == "1":
                jumlah = float(input("Masukkan jumlah setoran: "))
                setoran(jumlah)
            elif pilihan == "2":
                jumlah = float(input("Masukkan jumlah penarikan: "))
                penarikan(jumlah)
            elif pilihan == "3":
                cek_saldo()
            elif pilihan == "4":
                logout()
                break
            else:
                rospy.logwarn("Pilihan tidak valid! Silakan pilih 1, 2, 3, atau 4.")
        except ValueError:
            rospy.logwarn("Masukkan nilai yang valid.")

def setoran(jumlah):
    global saldo_pengguna
    if jumlah > 0:
        saldo_pengguna += jumlah
        rospy.loginfo(f"Setoran berhasil: {jumlah}")
        respon_transaksi.publish(f"Setoran berhasil: {jumlah}")
        arduino_pub.publish(1) 
    else:
        rospy.logwarn("Setoran gagal: Nilai tidak valid")
        respon_transaksi.publish("Setoran gagal: Nilai tidak valid")
        arduino_pub.publish(0) 

def penarikan(jumlah):
    global saldo_pengguna
    if jumlah <= 0:
        rospy.logwarn("Penarikan gagal: Nilai tidak valid")
        respon_transaksi.publish("Penarikan gagal: Nilai tidak valid")
        arduino_pub.publish(0)
    elif jumlah > saldo_pengguna:
        rospy.logwarn("Penarikan gagal: Saldo tidak mencukupi")
        respon_transaksi.publish("Penarikan gagal: Saldo tidak mencukupi")
        arduino_pub.publish(0)
    else:
        saldo_pengguna -= jumlah
        rospy.loginfo(f"Penarikan berhasil: {jumlah}")
        respon_transaksi.publish(f"Penarikan berhasil: {jumlah}")
        arduino_pub.publish(1) 

def cek_saldo():
    rospy.loginfo(f"Saldo Anda saat ini: {saldo_pengguna}")
    respon_transaksi.publish(f"Saldo Anda: {saldo_pengguna}")
    tampilkan_menu() 

def logout():
    global login_status
    login_status = False
    rospy.loginfo("Anda telah logout.")
    respon_transaksi.publish("Logout berhasil.")
    
    rospy.signal_shutdown()


if __name__ == '__main__':
    try:
        rospy.init_node('node_bank', anonymous=True)
        rospy.Subscriber('deteksi_login', Bool, proses_login)
        rospy.loginfo("Sistem bank sederhana dimulai...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Sistem bank dihentikan.")