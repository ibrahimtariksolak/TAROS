"""
Genisletilmis Kalman Filtresi — sabit hiz modeli, 6 durumlu.

Durum vektoru: [x, y, z, vx, vy, vz]
Olcum modelleri: radar (menzil, azimut, yukselis, Doppler),
EO/IR (azimut, yukselis), akustik (DOA azimut, DOA yukselis).
"""

import math

import numpy as np


class EKF:
    """Tek hedef icin genisletilmis Kalman filtresi."""

    SENSOR_RADAR = 'radar'
    SENSOR_EOIR = 'eoir'
    SENSOR_ACOUSTIC = 'acoustic'

    def __init__(self, state, covariance=None, process_noise_std=2.0):
        """
        Filtreyi baslangic durumu ve kovaryansiyla olusturur.

        state: 6 elemanli baslangic durumu [x, y, z, vx, vy, vz]
        covariance: 6x6 baslangic kovaryans matrisi (None = varsayilan)
        process_noise_std: ivme surec gurultusu standart sapmasi [m/s^2]
        """
        self.x = np.array(state, dtype=np.float64)
        if covariance is not None:
            self.P = np.array(covariance, dtype=np.float64)
        else:
            self.P = np.diag([100.0, 100.0, 100.0, 25.0, 25.0, 25.0])
        self._q_std = process_noise_std

    @property
    def position(self):
        """Konum tahmini [x, y, z]."""
        return self.x[:3].copy()

    @property
    def velocity(self):
        """Hiz tahmini [vx, vy, vz]."""
        return self.x[3:].copy()

    def predict(self, dt):
        """Durumu dt saniye ileri tasir."""
        F = self._transition_matrix(dt)
        Q = self._process_noise(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update_radar(self, measurement, R=None):
        """
        Radar olcumuyle gunceller.

        measurement: [menzil, azimut, yukselis, doppler_hiz]
        R: 4x4 olcum gurultu kovaryans matrisi
        """
        if R is None:
            R = np.diag([225.0, 0.001, 0.003, 1.0])
        z = np.array(measurement, dtype=np.float64)
        h, H = self._radar_model(self.x)
        self._ekf_update(z, h, H, R)

    def update_eoir(self, measurement, R=None):
        """
        EO/IR olcumuyle gunceller (sadece yonelim).

        measurement: [azimut, yukselis]
        R: 2x2 olcum gurultu kovaryans matrisi
        """
        if R is None:
            R = np.diag([1e-6, 1e-6])
        z = np.array(measurement, dtype=np.float64)
        h, H = self._bearing_model(self.x)
        self._ekf_update(z, h, H, R)

    def update_acoustic(self, measurement, R=None):
        """
        Akustik DOA olcumuyle gunceller.

        measurement: [doa_azimut, doa_yukselis]
        R: 2x2 olcum gurultu kovaryans matrisi
        """
        if R is None:
            R = np.diag([0.01, 0.02])
        z = np.array(measurement, dtype=np.float64)
        h, H = self._bearing_model(self.x)
        self._ekf_update(z, h, H, R)

    def innovation(self, measurement, sensor_type):
        """Olcum yeniligi ve yenilik kovaryansini hesaplar."""
        z = np.array(measurement, dtype=np.float64)
        if sensor_type == self.SENSOR_RADAR:
            h, H = self._radar_model(self.x)
            R = np.diag([225.0, 0.001, 0.003, 1.0])
        elif sensor_type == self.SENSOR_EOIR:
            h, H = self._bearing_model(self.x)
            R = np.diag([1e-6, 1e-6])
        else:
            h, H = self._bearing_model(self.x)
            R = np.diag([0.01, 0.02])

        y = z - h
        if sensor_type == self.SENSOR_RADAR:
            y[1] = self._wrap_angle(y[1])
            y[2] = self._wrap_angle(y[2])
        else:
            y[0] = self._wrap_angle(y[0])
            y[1] = self._wrap_angle(y[1])

        S = H @ self.P @ H.T + R
        return y, S

    def mahalanobis(self, measurement, sensor_type):
        """Olcumun Mahalanobis mesafesini hesaplar."""
        y, S = self.innovation(measurement, sensor_type)
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return float('inf')
        return float(y @ S_inv @ y)

    def _ekf_update(self, z, h, H, R):
        """EKF guncelleme adimi."""
        y = z - h

        if len(z) == 4:
            y[1] = self._wrap_angle(y[1])
            y[2] = self._wrap_angle(y[2])
        elif len(z) == 2:
            y[0] = self._wrap_angle(y[0])
            y[1] = self._wrap_angle(y[1])

        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        self.x = self.x + K @ y
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

    def _transition_matrix(self, dt):
        """Sabit hiz gecis matrisi."""
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        return F

    def _process_noise(self, dt):
        """Piecewise constant white noise ivme modeli."""
        q = self._q_std ** 2
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt

        Q = np.zeros((6, 6))
        for i in range(3):
            Q[i, i] = dt4 / 4.0 * q
            Q[i, i + 3] = dt3 / 2.0 * q
            Q[i + 3, i] = dt3 / 2.0 * q
            Q[i + 3, i + 3] = dt2 * q
        return Q

    def _radar_model(self, x):
        """
        Radar olcum modeli: h(x) ve Jacobian H.

        Cikti: [menzil, azimut, yukselis, radyal_hiz]
        """
        px, py, pz, vx, vy, vz = x
        r_horiz = math.sqrt(px * px + py * py)
        r = math.sqrt(px * px + py * py + pz * pz)

        if r < 1e-6:
            r = 1e-6
        if r_horiz < 1e-6:
            r_horiz = 1e-6

        az = math.atan2(py, px)
        el = math.atan2(pz, r_horiz)
        v_radial = (px * vx + py * vy + pz * vz) / r

        h = np.array([r, az, el, v_radial])

        H = np.zeros((4, 6))
        # dr/dx
        H[0, 0] = px / r
        H[0, 1] = py / r
        H[0, 2] = pz / r
        # daz/dx
        H[1, 0] = -py / (r_horiz * r_horiz)
        H[1, 1] = px / (r_horiz * r_horiz)
        # del/dx
        r_sq = r * r
        H[2, 0] = -px * pz / (r_sq * r_horiz)
        H[2, 1] = -py * pz / (r_sq * r_horiz)
        H[2, 2] = r_horiz / r_sq
        # dv_radial/dx (basitlestirilmis: sadece konum turevleri)
        H[3, 0] = vx / r - px * v_radial / r_sq
        H[3, 1] = vy / r - py * v_radial / r_sq
        H[3, 2] = vz / r - pz * v_radial / r_sq
        H[3, 3] = px / r
        H[3, 4] = py / r
        H[3, 5] = pz / r

        return h, H

    def _bearing_model(self, x):
        """
        Yonelim olcum modeli: h(x) ve Jacobian H.

        Cikti: [azimut, yukselis]
        """
        px, py, pz = x[0], x[1], x[2]
        r_horiz = math.sqrt(px * px + py * py)
        r = math.sqrt(px * px + py * py + pz * pz)

        if r < 1e-6:
            r = 1e-6
        if r_horiz < 1e-6:
            r_horiz = 1e-6

        az = math.atan2(py, px)
        el = math.atan2(pz, r_horiz)
        h = np.array([az, el])

        H = np.zeros((2, 6))
        H[0, 0] = -py / (r_horiz * r_horiz)
        H[0, 1] = px / (r_horiz * r_horiz)

        r_sq = r * r
        H[1, 0] = -px * pz / (r_sq * r_horiz)
        H[1, 1] = -py * pz / (r_sq * r_horiz)
        H[1, 2] = r_horiz / r_sq

        return h, H

    @staticmethod
    def _wrap_angle(angle):
        """Aciyi [-pi, pi] araligina sarar."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
