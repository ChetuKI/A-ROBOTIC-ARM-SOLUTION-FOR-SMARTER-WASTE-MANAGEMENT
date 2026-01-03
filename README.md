
# EXPLANATION

# 🤖 A Robotic Arm Solution for Smarter Waste Management

An intelligent, low‑cost waste segregation system that combines **deep learning**, **computer vision**, and **robotic automation** to identify and physically segregate waste materials in real time.

---

## 📌 Project Overview

This project uses a **YOLOv8n deep learning model** deployed on a **Raspberry Pi** to detect waste objects through a camera. Based on the detected waste category (metal, glass, or paper), a **6‑DOF robotic arm controlled by Arduino** performs automatic pick‑and‑place segregation.

The system reduces manual effort, improves recycling efficiency, and demonstrates a practical application of AI‑driven robotics for smart waste management.

---

## 🎯 Objectives

- Automate waste identification using deep learning   
- Physically segregate waste using a robotic arm  
- Reduce human involvement and sorting errors  
- Promote smart and sustainable waste management  

---

## 🛠️ Hardware Components

- Raspberry Pi 5
- Raspberry Pi Camera 
- Arduino Mega 
- 6‑DOF Robotic Arm  
- PCA9685 16‑Channel Servo Driver  
- Servo Motors (MG996R)  
- Power Supply and jumper wires  

---

## 💻 Software & Technologies

- **Programming Languages:** Python, Embedded C  
- **Deep Learning Model:** YOLOv8n  
- **Libraries:** OpenCV, PyTorch, NumPy, Ultralytics  
- **Training Platform:** Google Colab  
- **IDE & Tools:** Arduino IDE, VS Code  

---

## 🧠 Algorithm & Technique Used

- **Deep Learning (Object Detection)**
- YOLO (You Only Look Once) – YOLOv8n
- Serial communication between Raspberry Pi and Arduino

---

## 🔄 System Workflow

1. Camera captures live waste images  
2. YOLOv8n model detects and classifies waste  
3. Raspberry Pi sends class label to Arduino  
4. Robotic arm performs pick‑and‑place action  
5. Waste is placed into the correct bin  

---

## 📊 Model Training Details

- Platform: Google Colab  
- Epochs: ~60  
- Image Size: 640 × 640   
- Output: `best.pt` (trained model weights)

---

## 📦 Output

- Real‑time waste detection with bounding boxes
- Accurate classification with confidence scores
- Automatic physical segregation of waste
- Smooth robotic arm operation

---

## ⚠️ Limitations

- Supports lightweight waste objects only (≈200–300 g)
- Limited to predefined waste categories
- Performance depends on lighting conditions
  
---

## 🔮 Future Scope

- Add more waste categories (plastic, e‑waste, organic)
- Improve robustness under varying lighting
- Integrate conveyor belt systems
- Deploy in smart city waste management setups

---

## 👨‍💻 Contributors (Team Members)

- **Chetan Ingali**
- **Bhanu Kamble**
- **Chandrakant Acharatti**
- **Abhishek Kudari**
- **Dr. Arati Shahapurkar --Guide**

---

## 📜 License

This project is developed for **academic and educational purposes**.

---

⭐ If you found this project useful, feel free to star the repository!
