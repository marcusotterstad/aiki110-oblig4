Lokal IP på kablet nettverk på PC skal være 192.168.68.199 og bør komme automatisk ved tilkobling av
USB adapter da IP-adresse for kablet nettverk er satt statisk i Windows

PC på Rasperry Pi på kablet nettverk er konfigurert til å være 192.168.68.200

Pålogging via ssh er ssh hal@192.168.68.200 med passord dave

Filer can kopieres til PC ved scp hal@192.168.68.200:/home/hal/<fil> . 
eller fra PC ved scp <fil> hal@192.168.68.200:/home/hal

Nodesystemet kompileres med colcon build i katalogen ~/ros2
Merk at hvis det gjøres endringer på koden lokalt og man ønsker å teste oppdateringer til en gitt
node så må den gamle stoppes og en ny startes. Nodene er lett å se ved å gjøre ps -ef | grep ros2
Ved å kjøre kill <pid> på morprosessen (dvs den som har to pid verdier som er høye tall og ikke 1 i kolonne 2)
så vil man stoppe den spesifikke prosessen. Alternativt kan det lages et skript som stopper alle noder
og så starte systemet igjen med å kjøre ~/start_maott4996.sh