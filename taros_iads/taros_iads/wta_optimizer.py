"""Silah-hedef atama (WTA) optimizasyon modulu."""

import math


class DefenseUnit:
    """Tek bir savunma birligini temsil eder."""

    def __init__(self, unit_id, x, y, effective_range,
                 interceptor_count=10, operational=True):
        """Savunma birligini olusturur.

        Args:
            unit_id: Birlik tanimlayicisi.
            x: X konumu metre cinsinden.
            y: Y konumu metre cinsinden.
            effective_range: Efektif menzil metre cinsinden.
            interceptor_count: Kalan munisynon sayisi.
            operational: Birligin operasyonel olup olmadigi.
        """
        self.unit_id = unit_id
        self.x = x
        self.y = y
        self.effective_range = effective_range
        self.interceptor_count = interceptor_count
        self.operational = operational

    def distance_to(self, tx, ty):
        """Hedefe olan mesafeyi hesaplar."""
        return math.hypot(tx - self.x, ty - self.y)

    def can_engage(self, tx, ty):
        """Birligin hedefe ates edip edemeyecegini dondurur."""
        if not self.operational or self.interceptor_count <= 0:
            return False
        return self.distance_to(tx, ty) <= self.effective_range

    def success_probability(self, tx, ty):
        """Tahmini basari olasiligini hesaplar.

        Mesafe arttikca olasilik duser.
        """
        dist = self.distance_to(tx, ty)
        if dist > self.effective_range:
            return 0.0
        ratio = dist / self.effective_range
        return max(0.0, 1.0 - 0.5 * ratio * ratio)


class WTAOptimizer:
    """Acgozlu silah-hedef atama optimizasyonu."""

    def __init__(self, units):
        """Optimizer olusturur.

        Args:
            units: DefenseUnit listesi.
        """
        self._units = list(units)

    def assign(self, threats):
        """Tehditlere birlik atamasini yapar.

        Acgozlu yaklasimiyla her tehdide en yuksek basari olaslikli
        birligi atar. Bir birlik birden fazla hedefle eslesebilir.

        Args:
            threats: (track_id, x, y, priority) tuplelarindan olusan liste.
                priority degeri dusuk olan daha onceliklidir.

        Returns:
            Atama sonuc listesi: (track_id, unit_id, effector_type,
            priority, success_prob) tuplelari. Atanamayan tehditler
            listede yer almaz.
        """
        sorted_threats = sorted(threats, key=lambda t: t[3])
        assignments = []
        unit_loads = {u.unit_id: 0 for u in self._units}

        for track_id, tx, ty, priority in sorted_threats:
            best_unit = None
            best_prob = 0.0

            for unit in self._units:
                if not unit.can_engage(tx, ty):
                    continue
                prob = unit.success_probability(tx, ty)
                load_penalty = unit_loads[unit.unit_id] * 0.1
                score = prob - load_penalty
                if score > best_prob:
                    best_prob = score
                    best_unit = unit

            if best_unit is not None:
                assignments.append((
                    track_id,
                    best_unit.unit_id,
                    0,
                    priority,
                    best_unit.success_probability(tx, ty),
                ))
                unit_loads[best_unit.unit_id] += 1

        return assignments

    def compute_coverage(self, assignments, total_threats):
        """Atama kapsama oranini hesaplar.

        Args:
            assignments: assign() ciktisi.
            total_threats: Toplam tehdit sayisi.

        Returns:
            0.0-1.0 arasinda kapsama orani.
        """
        if total_threats == 0:
            return 1.0
        return len(assignments) / total_threats
