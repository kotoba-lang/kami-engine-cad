(ns kami.cad "Portable rational Bezier/NURBS-compatible CAD primitives.")
(defn curve [points weights]
  (when (or (< (count points) 2) (not= (count points) (count weights)) (some #(<= % 0) weights))
    (throw (ex-info "curve requires >=2 points and matching positive weights"
                    {:points (count points) :weights (count weights)})))
  {:cad/id (random-uuid) :cad/kind :nurbs-curve
   :cad/control-points (vec points) :cad/weights (vec weights)})
(defn- mix [a b t] (mapv #(+ %1 (* (- %2 %1) t)) a b))
(defn- homogeneous [{:cad/keys [control-points weights]}]
  (mapv (fn [[x y z] w] [(* x w) (* y w) (* z w) w]) control-points weights))
(defn- from-homogeneous [points]
  (curve (mapv (fn [[x y z w]] [(/ x w) (/ y w) (/ z w)]) points)
         (mapv #(nth % 3) points)))
(defn evaluate
  "Evaluate a rational Bezier curve with de Casteljau reduction. This is the
  first NURBS-compatible span evaluator; knots/loft compose on top."
  [{:cad/keys [control-points weights] :as c} t]
  (when-not (<= 0 t 1) (throw (ex-info "curve parameter must be within [0,1]" {:t t})))
  (let [h (homogeneous c)
        reduce1 (fn [pts] (mapv #(mix %1 %2 t) pts (subvec pts 1)))
        [x y z w] (loop [p h] (if (= 1 (count p)) (first p) (recur (reduce1 p))))]
    [(/ x w) (/ y w) (/ z w)]))

(defn split-curve
  "Split a rational Bezier span exactly at parameter t using homogeneous
  de Casteljau construction. Returns [left right]."
  [c t]
  (when-not (< 0 t 1) (throw (ex-info "split parameter must be within (0,1)" {:t t})))
  (let [levels (take-while seq (iterate #(mapv (fn [a b] (mix a b t)) % (subvec % 1))
                                         (homogeneous c)))
        left (mapv first levels)
        right (mapv last (reverse levels))]
    [(from-homogeneous left) (from-homogeneous right)]))

(defn trim-curve
  "Return the exact sub-curve spanning normalized parameters t0..t1."
  [c t0 t1]
  (when-not (<= 0 t0 t1 1)
    (throw (ex-info "trim range must satisfy 0 <= t0 <= t1 <= 1" {:t0 t0 :t1 t1})))
  (when (= t0 t1) (throw (ex-info "trim range cannot be empty" {:t t0})))
  (cond
    (and (zero? t0) (= 1 t1)) c
    (zero? t0) (first (split-curve c t1))
    (= 1 t1) (second (split-curve c t0))
    :else (let [[_ right] (split-curve c t0)
                local (/ (- t1 t0) (- 1 t0))]
            (first (split-curve right local)))))

(defn reverse-curve [c]
  (-> c
      (update :cad/control-points #(vec (reverse %)))
      (update :cad/weights #(vec (reverse %)))))

(defn- distance [a b]
  (#?(:clj Math/sqrt :cljs js/Math.sqrt)
   (reduce + (map (fn [x y] (let [d (- x y)] (* d d))) a b))))

(defn join-curves
  "Join ordered curve spans into a composite curve when adjacent endpoints
  meet within tolerance. Individual rational spans remain exact."
  ([curves] (join-curves curves 1.0e-6))
  ([curves tolerance]
   (when (< (count curves) 2) (throw (ex-info "join needs at least two curves" {})))
   (doseq [[a b] (partition 2 1 curves)]
     (let [gap (distance (evaluate a 1) (evaluate b 0))]
       (when (> gap tolerance)
         (throw (ex-info "curve endpoints exceed join tolerance" {:gap gap :tolerance tolerance})))))
   {:cad/id (random-uuid) :cad/kind :composite-curve :cad/segments (vec curves)}))

(defn evaluate-composite [{:cad/keys [segments]} t]
  (when-not (<= 0 t 1) (throw (ex-info "curve parameter must be within [0,1]" {:t t})))
  (let [n (count segments) scaled (* t n)
        index (min (dec n) (int (#?(:clj Math/floor :cljs js/Math.floor) scaled)))
        local (if (= t 1) 1 (- scaled index))]
    (evaluate (nth segments index) local)))

(defn tessellate [c segments]
  (when (< segments 1) (throw (ex-info "segments must be positive" {:segments segments})))
  (mapv #(evaluate c (/ % segments)) (range (inc segments))))

(defn snap-value [value increment]
  (if (pos? increment) (* increment #?(:clj (Math/round (double (/ value increment)))
                                        :cljs (js/Math.round (/ value increment)))) value))
(defn snap-point [point increment] (mapv #(snap-value % increment) point))

(defn move-control-point [c index point]
  (when-not (< -1 index (count (:cad/control-points c)))
    (throw (ex-info "control point index out of range" {:index index})))
  (assoc-in c [:cad/control-points index] (vec point)))

(defn set-weight [c index weight]
  (when-not (pos? weight) (throw (ex-info "weight must be positive" {:weight weight})))
  (assoc-in c [:cad/weights index] weight))

(defn loft
  "Create a sampled surface between compatible section curves."
  [curves u-segments]
  (when (< (count curves) 2) (throw (ex-info "loft needs two section curves" {})))
  {:cad/id (random-uuid) :cad/kind :loft-surface
   :cad/sections (mapv #(tessellate % u-segments) curves)})

(defn loft-mesh [{:cad/keys [sections]}]
  (let [rows (count sections) cols (count (first sections))
        positions (vec (mapcat identity sections))
        indices (vec (mapcat (fn [[r c]]
                               (let [a (+ (* r cols) c) b (inc a)
                                     d (+ (* (inc r) cols) c) e (inc d)]
                                 [a d b b d e]))
                             (for [r (range (dec rows)) c (range (dec cols))] [r c])))]
    {:positions positions :indices indices
     :normals (vec (repeat (count positions) [0 0 1]))}))
