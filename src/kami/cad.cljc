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
(defn curve-length
  "Adaptive rational-curve arc length with a world-unit error tolerance."
  ([c] (curve-length c 1.0e-6))
  ([c tolerance]
   (when-not (pos? tolerance) (throw (ex-info "length tolerance must be positive" {:tolerance tolerance})))
   (letfn [(measure [t0 p0 t1 p1 depth]
             (let [tm (/ (+ t0 t1) 2.0) pm (evaluate c tm)
                   chord (distance p0 p1) polygon (+ (distance p0 pm) (distance pm p1))]
               (if (or (>= depth 24) (<= (- polygon chord) tolerance))
                 polygon
                 (+ (measure t0 p0 tm pm (inc depth)) (measure tm pm t1 p1 (inc depth))))))]
     (measure 0.0 (evaluate c 0.0) 1.0 (evaluate c 1.0) 0))))
(defn bounds [points]
  (when-not (seq points) (throw (ex-info "bounds need points" {})))
  (let [axes (apply map vector points) minima (mapv #(reduce min %) axes) maxima (mapv #(reduce max %) axes)]
    {:min minima :max maxima :size (mapv - maxima minima)}))

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

;; Parametric 2D sketch constraints. Sketch coordinates are f64-compatible
;; scalar pairs and remain independent from display tessellation.
(defn sketch-point
  ([id x y] (sketch-point id x y false))
  ([id x y fixed?] {:sketch.point/id id :sketch.point/position [x y] :sketch.point/fixed? fixed?}))
(defn sketch-line [id a b] {:sketch.entity/id id :sketch.entity/kind :line :sketch.entity/a a :sketch.entity/b b})
(defn constraint [id kind data] (merge {:constraint/id id :constraint/kind kind} data))
(defn sketch [points entities constraints]
  {:sketch/points (into {} (map (juxt :sketch.point/id identity) points))
   :sketch/entities (vec entities) :sketch/constraints (vec constraints)})
(defn horizontal [id line-id] (constraint id :horizontal {:constraint/entity line-id}))
(defn vertical [id line-id] (constraint id :vertical {:constraint/entity line-id}))
(defn coincident [id point-a point-b] (constraint id :coincident {:constraint/a point-a :constraint/b point-b}))
(defn distance-constraint [id point-a point-b value]
  (when-not (pos? value) (throw (ex-info "distance constraint must be positive" {:value value})))
  (constraint id :distance {:constraint/a point-a :constraint/b point-b :constraint/value value}))

(defn- sketch-entity [s id] (first (filter #(= id (:sketch.entity/id %)) (:sketch/entities s))))
(defn- point-position [s id] (get-in s [:sketch/points id :sketch.point/position]))
(defn- fixed-point? [s id] (get-in s [:sketch/points id :sketch.point/fixed?]))
(defn- set-point [s id position]
  (if (fixed-point? s id) s (assoc-in s [:sketch/points id :sketch.point/position] (vec position))))
(defn- sqrt-value [x] #?(:clj (Math/sqrt x) :cljs (js/Math.sqrt x)))
(defn- distance2 [[ax ay] [bx by]] (sqrt-value (+ (* (- bx ax) (- bx ax)) (* (- by ay) (- by ay)))))
(defn- required-points [c entity]
  (case (:constraint/kind c)
    :horizontal [(:sketch.entity/a entity) (:sketch.entity/b entity)]
    :vertical [(:sketch.entity/a entity) (:sketch.entity/b entity)]
    [(:constraint/a c) (:constraint/b c)]))

(defn constraint-residual [s c]
  (let [entity (when-let [id (:constraint/entity c)] (sketch-entity s id))
        [a b] (required-points c entity) [ax ay] (point-position s a) [bx by] (point-position s b)]
    (case (:constraint/kind c)
      :horizontal (#?(:clj Math/abs :cljs js/Math.abs) (- by ay))
      :vertical (#?(:clj Math/abs :cljs js/Math.abs) (- bx ax))
      :coincident (distance2 [ax ay] [bx by])
      :distance (#?(:clj Math/abs :cljs js/Math.abs) (- (distance2 [ax ay] [bx by]) (:constraint/value c)))
      (throw (ex-info "unknown sketch constraint" {:constraint c})))))

(defn- solve-one [s c]
  (let [entity (when-let [id (:constraint/entity c)] (sketch-entity s id))
        [a b] (required-points c entity) pa (point-position s a) pb (point-position s b)
        fa (fixed-point? s a) fb (fixed-point? s b)
        place (fn [s target-a target-b]
                (cond (and fa fb) s fa (set-point s b target-b) fb (set-point s a target-a)
                      :else (-> s (set-point a target-a) (set-point b target-b))))]
    (case (:constraint/kind c)
      :horizontal (let [y (cond fa (second pa) fb (second pb) :else (/ (+ (second pa) (second pb)) 2))]
                    (place s [(first pa) y] [(first pb) y]))
      :vertical (let [x (cond fa (first pa) fb (first pb) :else (/ (+ (first pa) (first pb)) 2))]
                  (place s [x (second pa)] [x (second pb)]))
      :coincident (let [p (cond fa pa fb pb :else (mapv #(/ (+ %1 %2) 2) pa pb))] (place s p p))
      :distance (let [d (max 1.0e-12 (distance2 pa pb)) desired (:constraint/value c)
                      unit (mapv #(/ % d) (mapv - pb pa)) error (- d desired)
                      da (if fb error (/ error 2)) db (if fa error (/ error 2))]
                  (place s (mapv + pa (mapv #(* % da) unit))
                         (mapv - pb (mapv #(* % db) unit))))
      s)))

(defn solve-sketch
  ([s] (solve-sketch s {}))
  ([s {:keys [iterations tolerance] :or {iterations 64 tolerance 1.0e-7}}]
   (let [solved (loop [current s n 0]
                  (let [residuals (map #(constraint-residual current %) (:sketch/constraints current))]
                    (if (or (>= n iterations) (every? #(<= % tolerance) residuals)) current
                      (recur (reduce solve-one current (:sketch/constraints current)) (inc n)))))
         residuals (mapv #(constraint-residual solved %) (:sketch/constraints solved))]
     (assoc solved :sketch/solver {:converged? (every? #(<= % tolerance) residuals)
                                   :max-residual (reduce max 0 residuals) :residuals residuals}))))
