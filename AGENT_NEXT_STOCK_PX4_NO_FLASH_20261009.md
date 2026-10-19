# PROMPT AGENT BERIKUTNYA — PX4 STOCK FIRMWARE, TANPA FLASH (2026-10-09)

Anda adalah senior ROS2 Humble / MAVLink / PX4 Rover / LiDAR SLAM / safety engineer. **Kerjakan bertahap dengan alat otomasi pada device SIROBO**. Jangan hanya menjawab konsep: audit, implementasi pada dua workspace, pengujian, dan dokumentasi. Jangan menjalankan motor atau mengubah hardware FCU tanpa persetujuan lanjutan.

## TUJUAN YANG TIDAK BOLEH DIUBAH

1. Mission waypoint tetap milik Pixhawk/Cube firmware PX4 bawaan; RC/manual selalu mempunyai prioritas normal tertinggi; E-STOP fisik/failsafe independen lebih tinggi lagi.
2. MID-360S dan ROS2 memberi deteksi obstacle, pilihan kiri/kanan, wall follower, dan advisory berupa batas kecepatan longitudinal (`vx_limit`) dan koreksi belok (`yaw_correction`); Foxglove web untuk pemantauan dan tuning.
3. Mini-PC harus bisa terputus/tersambung (hotplug). Ketika mini-PC tidak tersedia, Pixhawk harus mempertahankan misi tersimpan dan RC/manual. **Jangan memaksakan kendaraan terus bergerak melewati rintangan yang belum terbukti aman.** AUTO-continue hanya saat firmware/posisi/failsafe dan bukti keamanan memadai. Bedakan misi *tetap tersimpan* dengan kendaraan *tetap bergerak*.
4. **WAJIB: firmware bawaan yang sudah ada tidak di-flash, tidak dimodifikasi, tidak diganti.** Jangan menjalankan `px4_uploader`, `make upload`, flash bootloader atau memakai binary FMU-v2/v3 hasil build sebagai firmware untuk Cube.
5. Jangan menyisipkan source patch ke PX4 produksi. Patches di `/media/sirobo/HDD/pixhawk/px4_firmware` dan checkout PX4 v1.17 adalah referensi eksperimen saja.
6. Jangan membuka kemampuan motion/mode/parameter/mission write pada hardware selama discovery. Jangan menulis parameter, meng-arm, mengubah mode, menjalankan mission fisik, atau memutar roda tanpa izin eksplisit.
7. Jangan klaim bahwa MAVLink TUNNEL custom `42001` bisa mengubah controller PX4 stock: firmware stock **tidak** otomatis menjalankan decoder/mixer yang pernah dibuat pada source patch.

## LOKASI PROYEK DAN KONDISI AWAL

- ROS2 LiDAR/P3-P6/Foxglove: `/media/sirobo/HDD/slam3d-livox-mid360s`
- ROS2 PX4 bridge/MAVLink: `/media/sirobo/HDD/pixhawk`
- Unified colcon cache: `/home/sirobo/.cache/mid360_pixhawk_ros`
- Bangun semua: `/media/sirobo/HDD/pixhawk/build.sh` atau script kedua proyek (keduanya mengarah ke cache gabungan).
- Source ROS2 kini 11 package; jangan hilangkan fungsi telemetry dan SLAM yang sudah ada.
- Backup parameter Cube asli: `/media/sirobo/HDD/pixhawk/backups/cube_px4_20261008_213146_complete.json` (732 parameter), backup mission dan parameter reversible ada di direktori yang sama. Semua backup read-only, jangan overwrite.
- Backup menunjukkan: `COM_OBS_AVOID` tidak tersedia; `CP_DIST` tidak tersedia; `MAV_FWDEXTSP=1`; `SYS_AUTOSTART=0`; `MAV_TYPE=0`; `COM_OF_LOSS_T=1`; `COM_OBL_RC_ACT=0`. Jangan menafsirkan suatu parameter sebagai dukungan fitur tanpa cek versi + uji.
- Status Cube sebelumnya: disarmed, airframe belum dikonfigurasi, posisi horizontal dan GPS belum valid. Tidak aman untuk AUTO rover fisik.
- Source eksperimen terdahulu: `avoidance_core` (CRC/sequence/hotplug test), `pixhawk_avoidance_adapter` (PointCloud2 to advisory), `pixhawk_bridge` (telemetry & TUNNEL opt-in). **Fitur TUNNEL custom tidak berguna sebagai koreksi motion di firmware STOCK** kecuali dibuktikan pesan tersebut didukung firmware.
- Pengujian yang pernah lulus hanyalah unit logic, pseudo-serial MAVLink simulator, P3/P4 offline, dan PX4 SITL v1.17. Tidak boleh disamakan dengan uji pada firmware yang kini terpasang di Cube.
- Firmware PX4 v1.17 berfitur mixer yang dicoba sebelumnya melebihi flash FMU-v2 dan FMU-v3. **Jangan lanjut menyiapkan flashing**.

## FASE 0 — IDENTIFIKASI FIRMWARE NYATA TANPA MENGUBAH ISI FCU

1. Baca `AUTOPILOT_VERSION` (flight_sw_version, middleware, board version/capabilities), `HEARTBEAT`, parameter read-only, mode serta protocol version dari jalur USB bridge. Hindari duplikasi akses serial; gunakan satu pemilik port dengan MAVLink routing read-only ke klien lain.
2. Periksa backup 732 param dan apakah firmware punya support rover Ackermann/AUTO mission/position estimate yang relevan. Jangan mengasumsikan dokumen PX4 v1.17 berlaku pada firmware terpasang.
3. Dokumentasikan `COM_OBS_AVOID`, `CP_DIST`, `MAV_FWDEXTSP`, `COM_RC_IN_MODE`, `COM_RC_OVERRIDE`, `COM_OF_LOSS_T`, `COM_OBL_RC_ACT`, supported flight modes, serta kompatibilitas MAVLink message/command berdasar source/tag firmware yang tepat.
4. Tuliskan capability matrix: **SUPPORTED / UNSUPPORTED / UNKNOWN / NOT_TESTED**, lengkap dengan bukti log/dokumen. Jangan mengubah parameter FCU.

## FASE 1 — BUKTIKAN APA YANG MUNGKIN DENGAN STOCK FIRMWARE

Uji berurutan pada SITL yang cocok versi/airframe. Jangan memaksakan solusi tertentu.

**Opsi A: native Mission Obstacle Avoidance** (preferensi jika sungguh tersedia)
- Beberapa PX4 lama mempunyai `COM_OBS_AVOID` / Path Planning Offboard Interface untuk Mission. Di backup Cube parameter ini TIDAK ada; verifikasi apakah firmware stock memang punya antarmuka sepadan.
- `OBSTACLE_DISTANCE` atau `DISTANCE_SENSOR` dapat menjadi telemetri/persepsi; jangan klaim PX4 Rover AUTO menyisipkan koreksi speed/yaw otomatis tanpa bukti actuator/mode.
- Uji kemampuan sebenarnya (AUTO tetap AUTO, waypoint tetap maju, kontrol belok/kecepatan benar-benar dipengaruhi) pada simulator. Jika tidak terbukti, tandai UNSUPPORTED dan hentikan opsi A.

**Opsi B: mission-aware native MAVLink waypoint/speed management**
- Kaji dukungan firmware untuk `MAV_CMD_DO_CHANGE_SPEED`, mission partial update, goto/reposition, mission pause/resume, dan mission index. Komando mission item tidak otomatis berarti bisa mengubah yaw setiap 100ms.
- Desain boleh membuat temporary bypass waypoint, menjaga mission snapshot/hash/index, dan rollback transaction. Tetapi uji apakah firmware menerima perubahan misi saat AUTO dan apakah hotplug mempertahankan jalur/keamanan.
- Jangan mengedit misi yang aktif atau menyisipkan waypoint ke Cube nyata sebelum simulasi, izin operator, dan backup diverifikasi.
- Jika jalur ini hanya menghasilkan perencanaan rute tingkat tinggi (bukan yaw mixing), sebut dengan jujur.

**Opsi C: AUTO → OFFBOARD/GUIDED/controlled mode → AUTO**
- Opsi fallback hanya jika firmware benar-benar mendukung mode rover yang dibutuhkan. Untuk Ackermann, MAVLink `SET_POSITION_TARGET_LOCAL_NED` yaw/yaw_rate bisa diabaikan dan arah diturunkan dari velocity vector; ROS2 rover-native speed+rate setpoint memerlukan interfaces sesuai firmware. Jangan mengirim yaw yang tidak didukung.
- FCU tetap menyimpan mission index; saat rejoin, uji checkpoint + ACK serta resume aman. Uji OFFBOARD proof-of-life, `COM_OF_LOSS_T`, RC takeover, dan loss behavior.
- Saat mini-PC hilang pada obstacle/unknown, **HOLD/STOP** lebih aman daripada meneruskan gerak. Jangan menjanjikan AUTO terus bergerak saat offboard terputus jika failsafe tidak memiliki kemampuan itu.

**Opsi D: hardware safety / output mixing independen** jika A-C tidak memenuhi semua kebutuhan
- Laporkan bahwa **true vx+yaw mixing di dalam PX4 AUTO stock tidak dapat diciptakan hanya dengan pesan TUNNEL biasa tanpa kode firmware pendukung**.
- Evaluasi perangkat safety independen (E-STOP hardwired, motor enable watchdog, range sensor langsung ke FCU jika didukung). Opsi output mixer eksternal berbeda dari mixing di PX4: butuh persetujuan arsitektur.
- Jangan menggunakan RC spoofing, manipulasi estimasi posisi, penjumlahan PWM aktuator, atau sumber joystick palsu sebagai jalan pintas untuk membelokkan AUTO Mission.

Buat tabel keputusan A-D: mendukung AUTO tanpa mode switch?, koreksi yaw real?, perubahan kecepatan?, misi tetap di FCU?, hotplug?, fail-safe?, RC override?, bukti, implementability versi firmware.

## FASE 2 — IMPLEMENTASI YANG DIPERBOLEHKAN DENGAN FIRMWARE STOCK

1. Adapter `stock_px4_adapter` atau modul terpisah di `/media/sirobo/HDD/pixhawk/src`, mengimpor `pixhawk_interfaces`; gunakan message dan command MAVLink yang benar-benar didukung firmware.
2. Node lokal di proyek LiDAR menghasilkan obstacle sectors, TTC, wall follower kiri/kanan, dan desired yaw/vx. Pisahkan obstacle perception, decision logic, mission supervisor, PX4 protocol adapter, web tuning, safety supervisor; ROS2 bukan authoritative motor controller.
3. Telemetry read-only mission current/reached, mode ACK, attitude yaw/yaw-rate, groundspeed, steering PWM **(PWM bukan sudut aktual)** dan RC status.
4. Batas input: time-sync NED↔ENU, FLU↔FRD, timestamp, stale sensor, nilai NaN/Inf, cloud valid-point count, TF, extrinsic. Kalibrasi sensor masih `uncalibrated`; jangan klaim valid dan jangan kirim hardware command.
5. Runtime command hardware OFF by default, dengan enable eksplisit setelah versi/parameter/mode/RC/hardwired E-STOP disetujui. Tahan jendela uji motor terpisah.
6. Foxglove web: `front_slow_m`, `front_stop_m`, TTC, `left/right_clearance_m`, `preferred_side`, `wall_kp/ki/kd`, yaw-rate/slew limits, speed/deceleration, radius belok Ackermann, `loss_policy`, `clear_confirm_frames`, `rc_priority`, status control mode, log keputusan, kandidat lintasan, heatmap obstacle, apply/rollback. Jangan izinkan web bypass safety.
7. Gunakan hanya satu build/cache ext4 pada `/home/sirobo/.cache/mid360_pixhawk_ros`. Jangan hapus output lama atau mengubah konfigurasi produksi tanpa kebutuhan.

## FASE 3 — FAILSAFE DAN HOTPLUG

Uji sedikitnya transisi:
- Mini-PC tidak tersedia sejak boot; PX4 dapat menjalankan mission normal **hanya setelah airframe+posisi valid** dan tidak bergantung pada ROS2 untuk navigasi.
- Mini-PC hadir saat jalan bebas; advisory tidak mengubah waypoint tak perlu.
- Ada rintangan: perlambat, STOP atau hindar jika jalur alternatif aman, ACK kontrol jelas.
- Cabut mini-PC ketika keadaan CLEAR: jika PX4 tetap AUTO dan mempunyai safe independent navigation/failsafe, mission boleh lanjut. Buktikan dengan simulator; jangan klaim blanket guarantee.
- Cabut mini-PC saat AVOID atau STOP: jangan hilangkan hazard latch di software yang hilang. Jika FCU stock tidak punya hazard watchdog/latch, syarat ini tidak terpenuhi tanpa safety hardware atau pergantian mode/failsafe; tandai BLOCKED.
- Sambung ulang ketika hazard belum hilang: jangan auto-rearm/resume; 3+ scan CLEAR, hysteresis dan operator approval setelah fault kritis.
- RC manual override lebih tinggi daripada perintah koreksi ROS; E-STOP motor independen lebih tinggi daripada RC.
- 0.3s, 1s, 5s, 30s loss; stale frames, duplicate commands, MAVLink connection resets, binary garbage, canceled command, mission progress conflict, reconnect 100 kali. Tunjukkan hasil dengan log dan assert.

## FASE 4 — UJI & ACCEPTANCE

- Unit tests untuk setiap state dan CRC/idempotence.
- SITL dengan **versi firmware yang setara atau kompatibilitasnya dibuktikan**.
- MAVLink pseudo-TTY simulator: mission index, ACK, mode, RC, servo PWM, dead-link, stale obstacle.
- ROS2 pointcloud replay MCAP dan fixture sintetik, Foxglove browser Playwright screenshot tiap mode, CPU/RAM p95, latency dan log MCAP.
- Gating fisik: PX4 stock hardware hanya boleh diuji dengan **read-only telemetry**, disarmed, tanpa motion/writes. Tidak ada flash, bootloader update, mission/parameter write tanpa izin. Simulasi PASS tidak sama dengan hardware motion PASS.
- Jangan klaim fitur berhasil jika data hardware belum ada: firmware existing menunjukkan `SYS_AUTOSTART=0` dan `MAV_TYPE=0`, GPS/posisi horizontal belum valid, sehingga AUTO Mission fisik belum siap.

## OUTPUT UNTUK USER

1. Implementasi source + tests + config dan launcher hanya untuk opsi yang terbukti dapat digunakan.
2. `/media/sirobo/HDD/pixhawk/STOCK_PX4_NO_FLASH_IMPLEMENTATION_AUDIT.md`: versi firmware, capability matrix, opsi A-D, arsitektur final, log uji, status blocker.
3. `/media/sirobo/HDD/slam3d-livox-mid360s/STOCK_PX4_FOXGLOVE_TUNING.md`: parameter, screenshot tiap mode, cara pemakaian.
4. Build semua ROS2 11+ package menggunakan satu cache dan laporkan jumlah paket PASS/FAIL.
5. Tabel fitur per status: IMPLEMENTED+TESTED SIM, IMPLEMENTED UNTESTED, BLOCKED firmware, BLOCKED calibration/hardware. Jelaskan secara gamblang syarat yang belum bisa dipenuhi tanpa modifikasi firmware/hardware.
6. Pastikan semua proses simulator berhenti bersih dan layanan sensor/PX4 read-only sebelumnya tidak rusak.

**Jangan abaikan batas fundamental:** PX4 stock yang tidak memiliki API intra-AUTO mixing tidak bisa disulap menerima `vx+yaw` mixer internal dengan hanya menerbitkan topik ROS atau TUNNEL. Agent harus menyelesaikan jalur terbaik yang memang tersedia, atau membuktikan ketidakmungkinan dan memberikan alternatif yang aman. Tidak boleh flash firmware.
