#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/cde/.local/lib/python3.8/site-packages')

import numpy as np
if not hasattr(np, '_core'):
    import numpy.core
    np._core = numpy.core
    sys.modules['numpy._core'] = numpy.core

import cv2
import pickle
import os
import glob
import re
from collections import defaultdict
import insightface

PACKAGE_PATH = '/home/cde/bobac4/src/yolo_face'
PHOTOS_DIR = os.path.join(PACKAGE_PATH, 'face_photos')
DB_PATH = os.path.join(PACKAGE_PATH, 'model', 'face_database.pkl')

recognizer = insightface.app.FaceAnalysis(name='buffalo_l')
recognizer.prepare(ctx_id=-1)

# 加载数据库
if os.path.exists(DB_PATH):
    with open(DB_PATH, 'rb') as f:
        db = pickle.load(f)
else:
    db = {}

# 按人名分组（张三_1.jpg, 张三_2.jpg → 张三）
photos = glob.glob(os.path.join(PHOTOS_DIR, '*.*'))
grouped = defaultdict(list)

for photo_path in photos:
    name = os.path.splitext(os.path.basename(photo_path))[0]
    # 去掉 _数字 后缀
    base_name = re.sub(r'_\d+$', '', name)
    grouped[base_name].append(photo_path)

print(f"找到 {len(grouped)} 个人，共 {len(photos)} 张照片\n")

for name, photo_list in grouped.items():
    embeddings = []
    
    for photo_path in photo_list:
        img = cv2.imread(photo_path)
        if img is None:
            print(f"  ❌ 无法读取: {os.path.basename(photo_path)}")
            continue
        
        faces = recognizer.get(img)
        if len(faces) == 0:
            print(f"  ❌ {os.path.basename(photo_path)}: 未检测到人脸")
            continue
        
        if len(faces) > 1:
            print(f"  ⚠ {os.path.basename(photo_path)}: 检测到 {len(faces)} 张人脸，取最大的")
        
        face = max(faces, key=lambda x: (x.bbox[2]-x.bbox[0])*(x.bbox[3]-x.bbox[1]))
        embeddings.append(face.normed_embedding)
        print(f"  ✅ {os.path.basename(photo_path)}: 提取成功")
    
    if embeddings:
        # 取平均特征
        avg_embedding = np.mean(embeddings, axis=0)
        db[name] = avg_embedding
        print(f"  🎯 {name}: 注册成功（{len(embeddings)}张照片平均）\n")
    else:
        print(f"  ❌ {name}: 没有有效的人脸\n")

with open(DB_PATH, 'wb') as f:
    pickle.dump(db, f, protocol=4)

print(f"数据库共有 {len(db)} 个身份: {list(db.keys())}")