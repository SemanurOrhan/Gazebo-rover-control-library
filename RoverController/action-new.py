from rover_controller import RoverController
import time

# RoverController örneği oluştur (UDP bağlantısı için)
rover = RoverController(connection_string='udpin:0.0.0.0:14550')

try:
    # Rover'ı başlat
    if rover.initialize():
        print("Rover başarıyla başlatıldı!")
        
        # İleri hareket
        rover.move_forward(speed=1.0, duration=5.0)
        print("Rover 5 saniye ileri hareket etti.")
        time.sleep(1)
        
        # Sağa dönüş (saat yönü, negatif angular speed)
        rover.turn(angular_speed=-0.5, duration=2.0)
        print("Rover 2 saniye sağa döndü.")
        time.sleep(1)
        
        # Geri hareket
        rover.move_backward(speed=1.0, duration=5.0)
        print("Rover 5 saniye geri hareket etti.")
        time.sleep(1)
        
        # Belirli bir konuma git
        rover.goto_position(target_x=5.0, target_y=2.0, speed=1.0, timeout=30)
        print("Konum navigasyonu tamamlandı.")
        
    else:
        print("Rover başlatılamadı.")
except KeyboardInterrupt:
    print("Kullanıcı tarafından durduruldu.")
finally:
    # Rover'ı disarm et ve bağlantıyı kapat
    rover.disarm()
    rover.close()