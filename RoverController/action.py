from rover_controller import RoverController
import time

# RoverController örneği oluştur (UDP bağlantısı için)
rover = RoverController(connection_string='udpin:0.0.0.0:14550')

try:
    # Rover'ı başlat
    if rover.initialize():
        print("Rover başarıyla başlatıldı!")
        # 1 m/s hızda 5 saniye ileri git
        rover.set_velocity(1.0, 0.0, 0.0, 5)
        print("Rover 5 saniye hareket etti.")
        # Durdur
        rover.set_velocity(0.0, 0.0, 0.0, 1)
        print("Rover durduruldu.")
    else:
        print("Rover başlatılamadı.")
except KeyboardInterrupt:
    print("Kullanıcı tarafından durduruldu.")
finally:
    # Rover'ı disarm et ve bağlantıyı kapat
    rover.disarm()
    rover.close()