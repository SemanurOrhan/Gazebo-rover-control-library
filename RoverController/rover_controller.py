from pymavlink import mavutil
import time
import sys
import math

class RoverController:
    def __init__(self, connection_string='udpin:0.0.0.0:14550', timeout=5):
        """
        RoverController sınıfı, Ardupilot SITL ile rover kontrolü için kullanılır.
        
        Args:
            connection_string (str): Bağlantı adresi (örn. 'udpin:0.0.0.0:14550' veya 'tcp:127.0.0.1:5760')
            timeout (int): Bağlantı zaman aşımı (saniye)
        """
        self.connection_string = connection_string
        self.timeout = timeout
        self.master = None
        self.connected = False
        self.position_tolerance = 0.5  # Konum hedefi için tolerans (metre)

    def connect(self):
        """Ardupilot SITL'e bağlan."""
        try:
            print(f"Bağlantı kuruluyor: {self.connection_string}")
            self.master = mavutil.mavlink_connection(self.connection_string, timeout=self.timeout)
            print("İlk HEARTBEAT bekleniyor...")
            self.master.wait_heartbeat(timeout=10)
            print(f"İlk HEARTBEAT alındı! Sistem ID: {self.master.target_system}, "
                  f"Component ID: {self.master.target_component}")
            self.connected = True
            return True
        except Exception as e:
            print(f"Bağlantı hatası: {e}")
            self.connected = False
            return False

    def get_current_mode(self):
        """Rover'ın mevcut modunu döndür."""
        try:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg:
                mode = mavutil.mode_string_v10(msg)
                print(f"Mevcut mod: {mode}")
                return mode
            else:
                print("HEARTBEAT mesajı alınamadı.")
                return None
        except Exception as e:
            print(f"Mod kontrol hatası: {e}")
            return None

    def set_mode(self, mode_id, timeout=10):
        """
        Rover'ın modunu değiştir.
        
        Args:
            mode_id (int): Mod ID (örn. GUIDED için 15)
            timeout (int): Zaman aşımı (saniye)
        
        Returns:
            bool: Mod değişimi başarılıysa True, değilse False
        """
        try:
            print(f"{mode_id} moduna geçiliyor...")
            self.master.set_mode(mode_id)
            start_time = time.time()
            while time.time() - start_time < timeout:
                msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg:
                    mode = mavutil.mode_string_v10(msg)
                    if mode == 'GUIDED' and mode_id == 15:
                        print("GUIDED moduna geçildi!")
                        return True
                time.sleep(0.1)
            print(f"{mode_id} mod zaman aşımına uğradı.")
            return False
        except Exception as e:
            print(f"Mod değiştirme hatası: {e}")
            return False

    def arm(self, timeout=10):
        """
        Rover'ı arm et.
        
        Args:
            timeout (int): Zaman aşımı (saniye)
        
        Returns:
            bool: Arm işlemi başarılıysa True, değilse False
        """
        try:
            print("Rover arm ediliyor...")
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0)
            start_time = time.time()
            while time.time() - start_time < timeout:
                msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
                if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if msg.result == 0:
                        print("Rover başarıyla arm edildi!")
                        return True
                    else:
                        print(f"Arm işlemi başarısız, sonuç: {msg.result}")
                        break
                time.sleep(0.1)
            print("Arm işlemi zaman aşımına uğradı.")
            return False
        except Exception as e:
            print(f"Arm hatası: {e}")
            return False

    def disarm(self):
        """Rover'ı disarm et."""
        try:
            print("Rover disarm ediliyor...")
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0)
            print("Rover disarm edildi.")
            return True
        except Exception as e:
            print(f"Disarm hatası: {e}")
            return False

    def set_velocity(self, vx, vy, vz, duration):
        """
        Rover'a hız komutu gönder.
        
        Args:
            vx (float): X ekseninde hız (m/s)
            vy (float): Y ekseninde hız (m/s)
            vz (float): Z ekseninde hız (m/s, genellikle 0)
            duration (float): Komut süresi (saniye)
        """
        try:
            print(f"Hız komutu gönderiliyor: vx={vx}, vy={vy}, vz={vz}")
            start_time = time.time()
            while time.time() - start_time < duration:
                self.master.mav.set_position_target_local_ned_send(
                    0, self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b0000111111000111, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)
                time.sleep(0.1)
        except Exception as e:
            print(f"Hız komutu hatası: {e}")

    def move_forward(self, speed=1.0, duration=5.0):
        """
        Rover'ı ileri hareket ettir.
        
        Args:
            speed (float): İleri hız (m/s)
            duration (float): Hareket süresi (saniye)
        """
        self.set_velocity(speed, 0.0, 0.0, duration)

    def move_backward(self, speed=1.0, duration=5.0):
        """
        Rover'ı geri hareket ettir.
        
        Args:
            speed (float): Geri hız (m/s)
            duration (float): Hareket süresi (saniye)
        """
        self.set_velocity(-speed, 0.0, 0.0, duration)

    def turn(self, angular_speed=0.5, duration=2.0):
        """
        Rover'ı yerinde döndür (yaw değişikliği).
        
        Args:
            angular_speed (float): Dönüş hızı (radyan/s, pozitif: saat yönünün tersi)
            duration (float): Dönüş süresi (saniye)
        """
        try:
            print(f"Dönüş komutu gönderiliyor: angular_speed={angular_speed}")
            start_time = time.time()
            while time.time() - start_time < duration:
                self.master.mav.set_position_target_local_ned_send(
                    0, self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b0000111110000111,  # Yalnızca yaw kontrolü
                    0, 0, 0, 0, 0, 0, 0, 0, 0, angular_speed, 0)
                time.sleep(0.1)
        except Exception as e:
            print(f"Dönüş hatası: {e}")

    def get_current_position(self):
        """
        Rover'ın mevcut yerel pozisyonunu döndür (x, y, z).
        
        Returns:
            tuple: (x, y, z) veya hata durumunda None
        """
        try:
            msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
            if msg:
                return (msg.x, msg.y, msg.z)
            else:
                print("Yerel pozisyon mesajı alınamadı.")
                return None
        except Exception as e:
            print(f"Yerel pozisyon alma hatası: {e}")
            return None

    def get_current_global_position(self):
        """
        Rover'ın mevcut global pozisyonunu döndür (enlem, boylam, yükseklik).
        
        Returns:
            tuple: (lat, lon, alt) veya hata durumunda None
        """
        try:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            if msg:
                lat = msg.lat / 1e7  # Enlem (derece)
                lon = msg.lon / 1e7  # Boylam (derece)
                alt = msg.alt / 1e3  # Yükseklik (metre)
                return (lat, lon, alt)
            else:
                print("Global pozisyon mesajı alınamadı.")
                return None
        except Exception as e:
            print(f"Global pozisyon alma hatası: {e}")
            return None

    def move_distance(self, distance, speed=1.0, direction='forward', timeout=30):
        """
        Rover'ı belirli bir mesafe boyunca hareket ettir.
        
        Args:
            distance (float): Hedef mesafe (metre)
            speed (float): Hareket hızı (m/s)
            direction (str): Hareket yönü ('forward' veya 'backward')
            timeout (int): Zaman aşımı (saniye)
        
        Returns:
            bool: Hedefe ulaşıldıysa True, değilse False
        """
        try:
            print(f"{distance} metre {direction} hareket ediliyor...")
            start_pos = self.get_current_position()
            if start_pos is None:
                print("Başlangıç pozisyonu alınamadı.")
                return False

            start_x, start_y, _ = start_pos
            traveled_distance = 0
            vx = speed if direction == 'forward' else -speed
            start_time = time.time()

            while traveled_distance < distance and (time.time() - start_time) < timeout:
                current_pos = self.get_current_position()
                if current_pos is None:
                    print("Mevcut pozisyon alınamadı, devam ediliyor...")
                    time.sleep(0.1)
                    continue

                current_x, current_y, _ = current_pos
                traveled_distance = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)

                if traveled_distance >= distance:
                    print(f"{distance} metre mesafeye ulaşıldı!")
                    self.set_velocity(0, 0, 0, 1)  # Durdur
                    return True

                self.set_velocity(vx, 0, 0, 0.1)
                time.sleep(0.1)

            print("Mesafe hareketi zaman aşımına uğradı.")
            self.set_velocity(0, 0, 0, 1)  # Durdur
            return False
        except Exception as e:
            print(f"Mesafe hareketi hatası: {e}")
            self.set_velocity(0, 0, 0, 1)  # Durdur
            return False

    def goto_position(self, target_x, target_y, speed=1.0, timeout=30):
        """
        Rover'ı belirli bir yerel konuma gönder.
        
        Args:
            target_x (float): Hedef x koordinatı (metre)
            target_y (float): Hedef y koordinatı (metre)
            speed (float): Hareket hızı (m/s)
            timeout (int): Zaman aşımı (saniye)
        
        Returns:
            bool: Hedefe ulaşıldıysa True, değilse False
        """
        try:
            print(f"Yerel konuma gidiliyor: x={target_x}, y={target_y}")
            start_time = time.time()
            while time.time() - start_time < timeout:
                current_pos = self.get_current_position()
                if current_pos is None:
                    print("Mevcut pozisyon alınamadı, devam ediliyor...")
                    time.sleep(0.1)
                    continue

                current_x, current_y, _ = current_pos
                dx = target_x - current_x
                dy = target_y - current_y
                distance = math.sqrt(dx**2 + dy**2)

                if distance < self.position_tolerance:
                    print("Yerel hedefe ulaşıldı!")
                    self.set_velocity(0, 0, 0, 1)  # Durdur
                    return True

                # Hız vektörünü hesapla
                angle = math.atan2(dy, dx)
                vx = speed * math.cos(angle)
                vy = speed * math.sin(angle)
                self.set_velocity(vx, vy, 0, 0.1)
                time.sleep(0.1)

            print("Yerel konuma ulaşma zaman aşımına uğradı.")
            self.set_velocity(0, 0, 0, 1)  # Durdur
            return False
        except Exception as e:
            print(f"Yerel konuma gitme hatası: {e}")
            self.set_velocity(0, 0, 0, 1)  # Durdur
            return False

    def goto_global_position(self, target_lat, target_lon, target_alt=0.0, speed=1.0, timeout=60):
        """
        Rover'ı belirli bir global konuma gönder (enlem, boylam, yükseklik).
        
        Args:
            target_lat (float): Hedef enlem (derece)
            target_lon (float): Hedef boylam (derece)
            target_alt (float): Hedef yükseklik (metre, genellikle 0)
            speed (float): Hareket hızı (m/s)
            timeout (int): Zaman aşımı (saniye)
        
        Returns:
            bool: Hedefe ulaşıldıysa True, değilse False
        """
        try:
            print(f"Global konuma gidiliyor: lat={target_lat}, lon={target_lon}, alt={target_alt}")
            start_time = time.time()
            while time.time() - start_time < timeout:
                current_pos = self.get_current_global_position()
                if current_pos is None:
                    print("Mevcut global pozisyon alınamadı, devam ediliyor...")
                    time.sleep(0.1)
                    continue

                current_lat, current_lon, current_alt = current_pos

                # Mesafeyi hesapla (Haversine formülü)
                R = 6371e3  # Dünya yarıçapı (metre)
                phi1 = math.radians(current_lat)
                phi2 = math.radians(target_lat)
                delta_phi = math.radians(target_lat - current_lat)
                delta_lambda = math.radians(target_lon - current_lon)

                a = math.sin(delta_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(delta_lambda/2)**2
                c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
                distance = R * c  # Metre cinsinden mesafe

                if distance < self.position_tolerance:
                    print("Global hedefe ulaşıldı!")
                    self.set_velocity(0, 0, 0, 1)  # Durdur
                    return True

                # Yönü hesapla
                theta = math.atan2(
                    math.sin(delta_lambda)*math.cos(phi2),
                    math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(delta_lambda)
                )
                theta = math.degrees(theta)  # Radyan -> derece
                theta = (theta + 360) % 360  # 0-360 aralığında

                # Yerel hız vektörlerini hesapla (NED frame)
                vx = speed * math.cos(math.radians(theta))
                vy = speed * math.sin(math.radians(theta))
                self.set_velocity(vx, vy, 0, 0.1)
                time.sleep(0.1)

            print("Global konuma ulaşma zaman aşımına uğradı.")
            self.set_velocity(0, 0, 0, 1)  # Durdur
            return False
        except Exception as e:
            print(f"Global konuma gitme hatası: {e}")
            self.set_velocity(0, 0, 0, 1)  # Durdur
            return False

    def close(self):
        """Bağlantıyı kapat."""
        if self.master:
            self.master.close()
            print("Bağlantı kapatıldı.")
            self.connected = False

    def initialize(self):
        """
        Rover'ı çalıştırılabilir hale getir (bağlan, GUIDED moda geç, arm et).
        
        Returns:
            bool: Başarılıysa True, değilse False
        """
        if not self.connect():
            return False
        if not self.set_mode(15):  # GUIDED mod
            return False
        if not self.arm():
            return False
        return True