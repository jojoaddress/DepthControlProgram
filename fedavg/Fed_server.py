# fed_server.py
import numpy as np
import json
from flask import Flask, request, jsonify
import os
import threading
import time

app = Flask(__name__)

# 存储各机具类型的全局模型
# 结构: {tool_type: {"theta": np.ndarray, "total_count": int}}
global_models = {}
MODEL_FILE = "global_models.npz"


def load_global_models():
    if os.path.exists(MODEL_FILE):
        data = np.load(MODEL_FILE)
        for key in data.files:
            if key.endswith("_theta"):
                tool = key[:-6]
                global_models[tool] = {
                    "theta": data[key],
                    "total_count": (
                        int(data[f"{tool}_count"]) if f"{tool}_count" in data else 0
                    ),
                }
        print("已加载全局模型")


def save_global_models():
    save_dict = {}
    for tool, m in global_models.items():
        save_dict[f"{tool}_theta"] = m["theta"]
        save_dict[f"{tool}_count"] = m["total_count"]
    np.savez(MODEL_FILE, **save_dict)
    print("全局模型已保存")


@app.route("/upload", methods=["POST"])
def upload():
    data = request.get_json()
    tool_type = data["tool_type"]
    local_theta = np.array(data["theta"])
    local_count = data["training_count"]

    if tool_type not in global_models:
        global_models[tool_type] = {"theta": local_theta, "total_count": local_count}
    else:
        # FedAvg 加权平均
        total = global_models[tool_type]["total_count"] + local_count
        w_global = global_models[tool_type]["total_count"] / total
        w_local = local_count / total
        global_models[tool_type]["theta"] = (
            w_global * global_models[tool_type]["theta"] + w_local * local_theta
        )
        global_models[tool_type]["total_count"] = total

    save_global_models()
    return jsonify(
        {"status": "ok", "new_total_count": global_models[tool_type]["total_count"]}
    )


@app.route("/download/<tool_type>", methods=["GET"])
def download(tool_type):
    if tool_type in global_models:
        return jsonify(
            {
                "theta": global_models[tool_type]["theta"].tolist(),
                "total_count": global_models[tool_type]["total_count"],
            }
        )
    else:
        return jsonify({}), 404


if __name__ == "__main__":
    load_global_models()
    app.run(host="0.0.0.0", port=5000, debug=False)
