from flask import Flask, request
import os
from datetime import datetime

app = Flask(__name__)
UPLOAD_FOLDER = 'received_images'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

@app.route('/upload', methods=['POST'])
def upload_image():
    if 'image' not in request.files:
        return 'No image uploaded', 400

    image = request.files['image']
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f'obstacle_{timestamp}.jpg'
    image_path = os.path.join(UPLOAD_FOLDER, filename)

    image.save(image_path)
    print(f"[SERVER] Obstacle detected. Image saved as: {filename}")

    return 'Image received', 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
