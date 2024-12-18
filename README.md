# **Multi-Drone Flood Detection Platform**  

This repository contains the implementation of a multi-drone-based flood detection platform designed to map flooded areas in real-time using computer vision. The platform aims to enhance safety and navigation for users and communities during severe weather.  

---

## **Getting Started**  

Follow this guide to set up and run the project.  

### **Prerequisites**  
Ensure the following are installed on your system:  
- Python 3.x  
- Node.js and npm (for the React frontend)  
- Git  

### **Installation**  

#### 1. Clone the Repository  
```bash  
git clone https://github.com/AndrewOOndara/drone-flood-detection.git  
cd drone-flood-detection  
```  

#### 2. Install Python Dependencies  
```bash  
pip install -r requirements.txt  
```  

#### 3. Set Up the React Frontend  
Navigate to the `react-app` directory:  
```bash  
cd react-app  
npm install  
```  

---

## **Usage**  

### **Run the Drone Simulation**  
From the project’s root directory, start the Python simulation:  
```bash  
python drone_simulation2.py  
```  

### **Run the Web App**  
Navigate to the `react-app` directory and start the React development server:  
```bash  
cd react-app  
npm start  
```  
The app will run at `http://localhost:3000`.  
