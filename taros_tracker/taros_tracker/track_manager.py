"""
Coklu hedef iz yoneticisi.

Iz yasam dongusu: TENTATIVE -> CONFIRMED -> COASTING -> DELETED.
M/N mantigi ile onay, zaman asimi ile silme.
"""

from taros_tracker.ekf import EKF

TENTATIVE = 0
CONFIRMED = 1
COASTING = 2
DELETED = 3

CONFIRM_HITS = 3
CONFIRM_WINDOW = 5
COASTING_TIMEOUT = 3.0
DELETE_TIMEOUT = 5.0


class ManagedTrack:
    """Tek hedefin iz durumunu ve EKF filtresini yonetir."""

    def __init__(self, track_id, initial_state, timestamp):
        """Yeni iz olusturur."""
        self.track_id = track_id
        self.ekf = EKF(initial_state)
        self.status = TENTATIVE
        self.hits = 1
        self.misses = 0
        self.total_updates = 1
        self.first_seen = timestamp
        self.last_update = timestamp
        self.sensor_hits = [0, 0, 0]
        self.classification_votes = {}
        self.classification = 'UNKNOWN'

    def record_hit(self, sensor_type, classification, timestamp):
        """Olcum eslesmesini kaydeder."""
        self.hits += 1
        self.total_updates += 1
        self.last_update = timestamp
        self.misses = 0

        sensor_idx = {'radar': 0, 'eoir': 1, 'acoustic': 2}.get(sensor_type, 0)
        self.sensor_hits[sensor_idx] += 1

        if classification and classification != 'UNKNOWN':
            self.classification_votes[classification] = (
                self.classification_votes.get(classification, 0) + 1)
            self.classification = max(
                self.classification_votes, key=self.classification_votes.get)

        if self.status == TENTATIVE and self.hits >= CONFIRM_HITS:
            self.status = CONFIRMED
        elif self.status == COASTING:
            self.status = CONFIRMED

    def record_miss(self):
        """Eslesmeme kaydeder."""
        self.misses += 1

    def check_timeout(self, current_time):
        """Zaman asimi kontrolu: coasting veya silme."""
        elapsed = current_time - self.last_update

        if self.status == CONFIRMED and elapsed > COASTING_TIMEOUT:
            self.status = COASTING
        elif self.status == COASTING and elapsed > DELETE_TIMEOUT:
            self.status = DELETED
        elif self.status == TENTATIVE and elapsed > COASTING_TIMEOUT:
            self.status = DELETED

    @property
    def sensor_contributions(self):
        """Her sensor tipinin normalize katki orani."""
        total = sum(self.sensor_hits)
        if total == 0:
            return [0.0, 0.0, 0.0]
        return [h / total for h in self.sensor_hits]

    @property
    def confidence(self):
        """Iz guvenilirlik skoru [0-1]."""
        if self.status == DELETED:
            return 0.0
        if self.status == COASTING:
            return 0.3
        if self.status == TENTATIVE:
            return 0.4

        n_sensors = sum(1 for h in self.sensor_hits if h > 0)
        base = 0.5 + 0.15 * n_sensors
        update_bonus = min(0.1, self.total_updates * 0.002)
        return min(1.0, base + update_bonus)


class TrackManager:
    """Coklu hedef iz yoneticisi."""

    def __init__(self):
        """Iz yoneticiyi baslatir."""
        self._tracks = {}
        self._next_id = 0

    @property
    def active_tracks(self):
        """Silinmemis tum izleri dondurur."""
        return [t for t in self._tracks.values() if t.status != DELETED]

    @property
    def confirmed_tracks(self):
        """Onayli izleri dondurur."""
        return [t for t in self._tracks.values() if t.status == CONFIRMED]

    def get_ekf_list(self):
        """Aktif izlerin EKF listesini dondurur (iliskilendirme icin)."""
        active = self.active_tracks
        return [t.ekf for t in active], active

    def predict_all(self, dt):
        """Tum aktif izleri dt kadar ileri tasir."""
        for track in self.active_tracks:
            track.ekf.predict(dt)

    def update_matched(self, track, measurement, sensor_type, classification, timestamp):
        """Eslesen iz-olcum ciftini gunceller."""
        if sensor_type == 'radar':
            track.ekf.update_radar(measurement)
        elif sensor_type == 'eoir':
            track.ekf.update_eoir(measurement)
        else:
            track.ekf.update_acoustic(measurement)

        track.record_hit(sensor_type, classification, timestamp)

    def initiate_track(self, initial_state, timestamp):
        """Yeni iz olusturur ve dondurur."""
        track_id = self._next_id
        self._next_id += 1
        track = ManagedTrack(track_id, initial_state, timestamp)
        self._tracks[track_id] = track
        return track

    def maintain(self, current_time):
        """Zaman asimi kontrolu ve silinen izleri temizler."""
        for track in list(self._tracks.values()):
            track.check_timeout(current_time)

        deleted = [tid for tid, t in self._tracks.items() if t.status == DELETED]
        for tid in deleted:
            del self._tracks[tid]

    def mark_unmatched(self, tracks):
        """Eslesmeyen izlere miss kaydeder."""
        for track in tracks:
            track.record_miss()
