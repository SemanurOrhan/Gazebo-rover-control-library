from rover_controller import RoverController
import time

rover = RoverController(connection_string='udpin:0.0.0.0:14550')

try:
    # Rover'ı başlat
    if rover.initialize():
        print("Rover başarıyla başlatıldı!")
        
        # 5 metre ileri git
        rover.move_distance(distance=5.0, speed=1.0, direction='forward', timeout=30)
        print("Rover 5 metre ileri hareket etti.")
        time.sleep(1)
        
        # Sağa dönüş (saat yönü, negatif angular speed)
        rover.turn(angular_speed=-0.5, duration=2.0)
        print("Rover 2 saniye sağa döndü.")
        time.sleep(1)
        
        # Mevcut global pozisyonu al
        current_pos = rover.get_current_global_position()
        if current_pos:
            current_lat, current_lon, current_alt = current_pos
            print(f"Mevcut global pozisyon: lat={current_lat}, lon={current_lon}, alt={current_alt}")
            
            # 10 metre ileri bir global konuma git (örnek olarak küçük bir enlem/boylam değişikliği)
            # 1 derece enlem/boylam yaklaşık 111 km, 10 metre için ~0.00009 derece
            target_lat = -35.36284218 
            target_lon = 149.16515666
            rover.goto_global_position(target_lat, target_lon, 0.0, speed=1.0, timeout=60)
            print("Global konum navigasyonu tamamlandı.")
        else:
            print("Global pozisyon alınamadı, navigasyon atlandı.")
        
    else:
        print("Rover başlatılamadı.")
except KeyboardInterrupt:
    print("Kullanıcı tarafından durduruldu.")
finally:
    # Rover'ı disarm et ve bağlantıyı kapat
    rover.disarm()
    rover.close()