# cloud_cellular_image_uploader
# :czech_republic:

Jedná se o ROS2 uzly, které umožňují odesílání obrazových dat přes LTE síť na různá cloudová úložiště. Uzly přijímají obrazová data z témata `/image_raw/compressed` a následně je pomocí API jednotlivých cloudových platforem odesílá na jejich úložiště.
V aktuálním stavu je uzel pro odesílání snímků na Firebase schopný odeslat cca 8 snímků/sekundu v rozlišení 480p a uzel pro odesílání na BotX přibližně 1 snímek/sekundu ve stejném rozlišení.
### Poznámka:
Pro použití ve vlastním projektu doporučuji využít implementaci pro Firebase - kvůli lepší dokumentaci Firebase a celkové funkčnosti. 
Implementace pro BotX byla zprovozněna jako "proof of concept" - toho, že je toto řešení universálně použitelné pro více cloudových úložišť. 

## Prerekvizity:
 - Uzly přijímají relevantní data z témata `/image_raw/compressed` , proto je třeba použít [v4l2_camera](https://github.com/tier4/ros2_v4l2_camera?tab=readme-ov-file#v4l2_camera) včetně knihovny `image_transport`

## Zprovoznění:
### Firebase:
- V konzoli Firebase je třeba založit "webovou" aplikaci.
- V záložce "storage" zkopírovat url adresu vašeho úložiště a vložit na připravené místo v kódu uzlu.
- V záložce "Project settings -> Service accounts" vytvořit nový privátní klíč, který uložíte na stejné zařízení, na kterém uzel poběží. Umístění tohoto souboru následně vložte na připravené místo v kódu uzlu.

   Nyní na zařízení můžete spustit v4l2_camera, sender_node_fire a sledovat jak v úložišti přibívají snímky.

### BotX:
- V záložce "Data & Files" vytvořte složku, do které se budou snímky odesílat. Url složky vložte do připraveného místa v kódu uzlu.
- Na připravené místo v uzlu také přidejte své přihlašovací údaje - uzel na jejich základě získává JWT token.

  Nyní na zařízení můžete spustit v4l2_camera, sender_node_botx a sledovat jak v úložišti přibívají snímky.

# :us:/:gb:

These are ROS2 nodes enabling the transmission of image data over LTE networks to various cloud storage platforms. The nodes receive image data from the topic `/image_raw/compressed` and then send it to their respective storage using the APIs of individual cloud platforms.
Currently, the node for sending images to Firebase is capable of sending approximately 8 images per second at a resolution of 480p, while the node for sending to BotX can manage about 1 image per second at the same resolution.

### Note:
For use in your own project, it is recommended to utilize the Firebase implementation due to better Firebase documentation and overall functionality. The BotX implementation was established as a proof of concept, demonstrating the universality of this solution for multiple cloud storage platforms.

## Prerequisites:
- The nodes receive relevant data from the topic `/image_raw/compressed`, so you need to use [v4l2_camera](https://github.com/tier4/ros2_v4l2_camera?tab=readme-ov-file#v4l2_camera) including the `image_transport` library.

## Setup:
### Firebase:
- In the Firebase console, you need to create a "web" application.
- In the "storage" tab, copy the URL of your storage and paste it into the designated place in the node code.
- In the "Project settings -> Service accounts" tab, create a new private key, which you then save on the same device where the node will run. Then, insert the location of this file into the designated place in the node code.

   Now, you can start v4l2_camera, sender_node_fire, and observe images accumulating in the storage.

### BotX:
- In the "Data & Files" tab, create a folder where the images will be sent. Insert the folder URL into the designated place in the node code.
- Also, add your login credentials to the designated place in the node code - the node uses them to obtain a JWT token.

   Now, you can start v4l2_camera, sender_node_botx, and observe images accumulating in the storage.
