import numpy as np
from sklearn.linear_model import LinearRegression
import argparse
import matplotlib.pyplot as plt

# argparse를 사용하여 파일 경로 입력 받기
parser = argparse.ArgumentParser(description="Process some integers.")
parser.add_argument("path", type=str, help="Path to the data file")
path = "/home/mint/mint_ws/"
file_name = "HYWC_linear_0819_3_residual.txt"
args = parser.parse_args([path + file_name])

# 마커 사이즈 및 선 굵기
marker_size = 11
line_width = 4

# 파일에서 데이터 읽기
timestamp = []
residuals = []
covariances = []

with open(args.path, "r") as f:
    for line in f:
        t, r, c = map(float, line.strip().split(","))
        timestamp.append(t)
        residuals.append(r)
        covariances.append(c)

# 데이터를 numpy 배열로 변환
timestamp = np.array(timestamp).reshape(-1, 1)
timestamp = timestamp - timestamp[0]
residuals = np.array(residuals).reshape(-1, 1)
covariances = np.array(covariances).reshape(-1, 1)

# GNSS mode별 데이터 분리
rtk_fix = covariances <= 0.01
rtk_float = covariances > 1
dgps = (0.01 < covariances) & (covariances <= 1)

# 회귀 모델 수행
X = timestamp
reg = LinearRegression().fit(X, residuals)
y_pred = reg.predict(X)

# 그래프 그리기
plt.figure()
plt.plot(
    timestamp[rtk_fix],
    residuals[rtk_fix],
    "g.",
    label="RTK-Fix",
    markersize=marker_size,
)
plt.plot(
    timestamp[rtk_float],
    residuals[rtk_float],
    "b.",
    label="RTK-Float",
    markersize=marker_size,
)
plt.plot(timestamp[dgps], residuals[dgps], "r.", label="DGNSS", markersize=marker_size)
plt.plot(timestamp, y_pred, "c-", label="Regression", linewidth=line_width)

plt.xlabel(r"$Time \; [s]$", fontsize=16)
plt.ylabel(r"$b \; [m]$", fontsize=16)
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.xlim(0, timestamp[-1])
plt.ylim(-0.05, 0.05)
plt.locator_params(axis="y", nbins=5)
plt.gca().yaxis.set_major_locator(plt.MultipleLocator(0.025))
plt.grid(True)
plt.legend(fontsize=14)
plt.tight_layout()
plt.savefig(f"{file_name.split('.')[0]}.pdf")

plt.show()

# 기본 정보 출력
print(f"데이터 수: {len(timestamp)}")
print(f"Residual 평균: {np.mean(residuals):.3f}")
print(f"Residual 표준편차: {np.std(residuals):.3f}")
print(f"회귀 기울기: {reg.coef_[0][0]:.3f}")
print(f"회귀 절편: {reg.intercept_[0]:.3f}")
