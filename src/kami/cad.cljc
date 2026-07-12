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

;; Sketch-corner fillet/chamfer. Both operate on a straight-line-to-straight-
;; line corner, trimming the shared vertex back to two tangent points and
;; splicing in a replacement arc (fillet) or straight cut (chamfer) entity so
;; the profile stays a single closed loop for `sketch-loop->polygon`.
(defn sketch-arc [id a b center]
  {:sketch.entity/id id :sketch.entity/kind :arc :sketch.entity/a a :sketch.entity/b b
   :sketch.entity/center (vec center)})

(defn- vsub [a b] (mapv - a b))
(defn- vadd [a b] (mapv + a b))
(defn- vscale [v k] (mapv #(* % k) v))
(defn- vlen [v] (sqrt-value (reduce + (map #(* % %) v))))
(defn- vnorm [v]
  (let [len (vlen v)]
    (when (< len 1.0e-9) (throw (ex-info "corner edge has zero length" {:vector v})))
    (vscale v (/ 1.0 len))))
(defn- vdot [a b] (reduce + (map * a b)))

(defn- corner-lines
  "The two straight entities meeting at `corner`: one ending there
  (`:incoming`), one starting there (`:outgoing`). Anything else (a T-joint,
  an arc, an unclosed end) is out of scope and rejected."
  [s corner]
  (let [touching (filter #(and (= :line (:sketch.entity/kind %))
                                (or (= corner (:sketch.entity/a %)) (= corner (:sketch.entity/b %))))
                         (:sketch/entities s))
        incoming (first (filter #(= corner (:sketch.entity/b %)) touching))
        outgoing (first (filter #(= corner (:sketch.entity/a %)) touching))]
    (when-not (and (= 2 (count touching)) incoming outgoing (not= incoming outgoing))
      (throw (ex-info "fillet/chamfer corner must join exactly one incoming and one outgoing straight line"
                      {:corner corner :touching (count touching)})))
    [incoming outgoing]))

(defn- corner-geometry [s corner]
  (let [[incoming outgoing] (corner-lines s corner)
        corner-position (point-position s corner)
        a (point-position s (:sketch.entity/a incoming))
        b (point-position s (:sketch.entity/b outgoing))
        len-ca (vlen (vsub a corner-position))
        len-cb (vlen (vsub b corner-position))
        u (vnorm (vsub a corner-position))
        v (vnorm (vsub b corner-position))
        theta (#?(:clj Math/acos :cljs js/Math.acos) (max -1.0 (min 1.0 (vdot u v))))]
    (when (< theta 1.0e-6) (throw (ex-info "corner edges are collinear" {:corner corner})))
    {:incoming incoming :outgoing outgoing :corner-position corner-position
     :u u :v v :len-ca len-ca :len-cb len-cb :theta theta}))

(defn- splice-corner [s incoming outgoing start-id start-point end-id end-point replacement]
  (-> s
      (assoc-in [:sketch/points start-id] (sketch-point start-id (first start-point) (second start-point) true))
      (assoc-in [:sketch/points end-id] (sketch-point end-id (first end-point) (second end-point) true))
      (update :sketch/entities
              (fn [entities] (mapv (fn [e] (cond (= (:sketch.entity/id incoming) (:sketch.entity/id e)) (assoc e :sketch.entity/b start-id)
                                                 (= (:sketch.entity/id outgoing) (:sketch.entity/id e)) (assoc e :sketch.entity/a end-id)
                                                 :else e))
                                   entities)))
      (update :sketch/entities conj replacement)))

(defn fillet-sketch
  "Round the corner at `corner` with a tangent arc of `radius`, replacing the
  shared vertex with trim points `start-id`/`end-id` and arc `arc-id`."
  [s corner radius start-id end-id arc-id]
  (when-not (pos? radius) (throw (ex-info "fillet radius must be positive" {:radius radius})))
  (let [{:keys [incoming outgoing corner-position u v len-ca len-cb theta]} (corner-geometry s corner)
        tangent-length (/ radius (#?(:clj Math/tan :cljs js/Math.tan) (/ theta 2)))]
    (when (or (>= tangent-length len-ca) (>= tangent-length len-cb))
      (throw (ex-info "fillet radius exceeds adjacent edge length"
                      {:radius radius :tangent-length tangent-length :len-ca len-ca :len-cb len-cb})))
    (let [start-point (vadd corner-position (vscale u tangent-length))
          end-point (vadd corner-position (vscale v tangent-length))
          bisector (vnorm (vadd u v))
          center (vadd corner-position (vscale bisector (/ radius (#?(:clj Math/sin :cljs js/Math.sin) (/ theta 2)))))]
      (splice-corner s incoming outgoing start-id start-point end-id end-point
                     (sketch-arc arc-id start-id end-id center)))))

(defn chamfer-sketch
  "Cut the corner at `corner` with a straight chamfer of `distance` measured
  along each incident edge, replacing the shared vertex with trim points
  `start-id`/`end-id` and chamfer segment `chamfer-id`."
  [s corner distance start-id end-id chamfer-id]
  (when-not (pos? distance) (throw (ex-info "chamfer distance must be positive" {:distance distance})))
  (let [{:keys [incoming outgoing corner-position u v len-ca len-cb]} (corner-geometry s corner)]
    (when (or (>= distance len-ca) (>= distance len-cb))
      (throw (ex-info "chamfer distance exceeds adjacent edge length"
                      {:distance distance :len-ca len-ca :len-cb len-cb})))
    (let [start-point (vadd corner-position (vscale u distance))
          end-point (vadd corner-position (vscale v distance))]
      (splice-corner s incoming outgoing start-id start-point end-id end-point
                     (sketch-line chamfer-id start-id end-id)))))

(defn- tessellate-entity
  "Sample one sketch entity into its 2D points, excluding the trailing
  endpoint so closed loops concatenate without duplicate vertices."
  [s entity segments]
  (case (:sketch.entity/kind entity)
    :line [(point-position s (:sketch.entity/a entity))]
    :arc (let [center (:sketch.entity/center entity)
               p1 (point-position s (:sketch.entity/a entity))
               p2 (point-position s (:sketch.entity/b entity))
               r (vlen (vsub p1 center))
               atan2 (fn [[x y]] #?(:clj (Math/atan2 y x) :cljs (js/Math.atan2 y x)))
               pi #?(:clj Math/PI :cljs js/Math.PI)
               a1 (atan2 (vsub p1 center)) a2 (atan2 (vsub p2 center))
               raw-delta (- a2 a1)
               delta (cond (> raw-delta pi) (- raw-delta (* 2 pi))
                           (< raw-delta (- pi)) (+ raw-delta (* 2 pi))
                           :else raw-delta)]
           (mapv (fn [i] (let [angle (+ a1 (* delta (/ i segments)))]
                           (vadd center (vscale [(#?(:clj Math/cos :cljs js/Math.cos) angle)
                                                  (#?(:clj Math/sin :cljs js/Math.sin) angle)] r))))
                 (range segments)))
    (throw (ex-info "unsupported sketch entity for tessellation" {:kind (:sketch.entity/kind entity)}))))

(defn sketch-loop->polygon
  "Walk a sketch's entities as a single closed loop (each entity's `:b`
  matching the next entity's `:a`) into a flat 3D polygon point list
  consumable by `extrude-polygon`, tessellating arcs along the way."
  ([s] (sketch-loop->polygon s 16))
  ([s segments]
   (let [entities (:sketch/entities s)
         by-start (into {} (map (juxt :sketch.entity/a identity) entities))]
     (when-not (= (count entities) (count by-start))
       (throw (ex-info "sketch entities must form a single closed loop (one edge per start point)" {})))
     (let [start-id (:sketch.entity/a (first entities))
           points (loop [current-id start-id acc [] steps 0]
                    (if (and (pos? steps) (= current-id start-id))
                      acc
                      (do (when (> steps (count entities))
                            (throw (ex-info "sketch loop never returns to its start point" {:start start-id})))
                          (let [entity (get by-start current-id)]
                            (when-not entity (throw (ex-info "sketch loop is not closed" {:missing-start current-id})))
                            (recur (:sketch.entity/b entity)
                                   (into acc (tessellate-entity s entity segments))
                                   (inc steps))))))]
       (mapv (fn [[x y]] [x y 0]) points)))))

;; Ordered parametric feature history. Features refer only to earlier feature
;; ids, making rebuild results deterministic and project-file portable.
(defn feature
  ([id kind inputs params] (feature id kind inputs params false))
  ([id kind inputs params suppressed?]
   {:feature/id id :feature/kind kind :feature/inputs (vec inputs)
    :feature/params params :feature/suppressed? (boolean suppressed?)}))

(defn feature-model [features]
  (let [features (vec features) ids (mapv :feature/id features)]
    (when-not (= (count ids) (count (set ids)))
      (throw (ex-info "duplicate feature id" {:ids ids})))
    {:feature-model/features features}))

(defn update-feature [model id f & args]
  (when-not (some #(= id (:feature/id %)) (:feature-model/features model))
    (throw (ex-info "feature not found" {:id id})))
  (update model :feature-model/features
          #(mapv (fn [feature] (if (= id (:feature/id feature)) (apply f feature args) feature)) %)))
(defn suppress-feature [model id suppressed?]
  (update-feature model id assoc :feature/suppressed? (boolean suppressed?)))

(declare extrude-polygon boolean-union boolean-difference boolean-intersect)
(defn- evaluate-feature [feature inputs]
  (let [p (:feature/params feature)]
    (case (:feature/kind feature)
      :source (:value p)
      :move-control-point (move-control-point (first inputs) (:index p) (:point p))
      :set-weight (set-weight (first inputs) (:index p) (:weight p))
      :trim (trim-curve (first inputs) (:t0 p) (:t1 p))
      :reverse (reverse-curve (first inputs))
      :join (join-curves inputs (:tolerance p 1.0e-6))
      :loft (loft inputs (:segments p 32))
      :fillet-sketch (fillet-sketch (first inputs) (:corner p) (:radius p) (:start-id p) (:end-id p) (:arc-id p))
      :chamfer-sketch (chamfer-sketch (first inputs) (:corner p) (:distance p) (:start-id p) (:end-id p) (:chamfer-id p))
      :sketch->polygon (sketch-loop->polygon (first inputs) (:segments p 16))
      :extrude (extrude-polygon (first inputs) (:direction p))
      ;; Boolean feature kinds always evaluate to a *vector* of 0..2 result
      ;; solids (see `boolean-union`/`boolean-difference`/`boolean-intersect`
      ;; docstrings) -- consumers that need a single mesh must pick an
      ;; element (typically `(first result)`) and check `(seq result)`.
      :boolean-union (boolean-union (first inputs) (second inputs))
      :boolean-difference (boolean-difference (first inputs) (second inputs))
      :boolean-intersect (boolean-intersect (first inputs) (second inputs))
      (throw (ex-info "unsupported CAD feature" {:kind (:feature/kind feature)})))))

(defn recompute-feature-model
  "Rebuild an ordered feature tree. A failed feature and every dependent
  feature receive explicit status; independent later branches still rebuild."
  [model]
  (loop [remaining (:feature-model/features model) results {} statuses {} seen #{}]
    (if-let [f (first remaining)]
      (let [id (:feature/id f) input-ids (:feature/inputs f)
            missing (seq (remove seen input-ids))
            failed-input (first (filter #(not= :ok (get-in statuses [% :status])) input-ids))]
        (cond
          (:feature/suppressed? f)
          (recur (rest remaining) results (assoc statuses id {:status :suppressed}) (conj seen id))
          missing
          (recur (rest remaining) results
                 (assoc statuses id {:status :error :reason :input-not-before-feature :inputs (vec missing)}) (conj seen id))
          failed-input
          (recur (rest remaining) results
                 (assoc statuses id {:status :blocked :reason :failed-input :input failed-input}) (conj seen id))
          :else
          (let [outcome (try {:value (evaluate-feature f (mapv results input-ids))}
                             (catch #?(:clj Exception :cljs :default) e
                               {:error #?(:clj (.getMessage e) :cljs (.-message e))}))]
            (if-let [message (:error outcome)]
              (recur (rest remaining) results
                     (assoc statuses id {:status :error :reason :evaluation-failed :message message}) (conj seen id))
              (recur (rest remaining) (assoc results id (:value outcome))
                     (assoc statuses id {:status :ok}) (conj seen id))))))
      (assoc model :feature-model/results results :feature-model/statuses statuses))))

;; Boundary-representation-compatible polygon extrusion. Faces retain their
;; polygon loops; triangulation is a renderer/export concern.
(defn solid [vertices faces]
  {:cad/id (random-uuid) :cad/kind :solid :solid/vertices (mapv vec vertices)
   :solid/faces (mapv vec faces)})
(defn solid-edges [s]
  (->> (:solid/faces s)
       (mapcat #(map vector % (concat (rest %) [(first %)])))
       (map #(vec (sort %))) frequencies))
(defn watertight-solid? [s]
  (let [vertices (:solid/vertices s) faces (:solid/faces s)]
    (and (seq vertices) (seq faces)
         (every? #(and (>= (count %) 3)
                       (every? (fn [i] (and (integer? i) (<= 0 i (dec (count vertices))))) %)) faces)
         (every? #(= 2 %) (vals (solid-edges s))))))

(defn extrude-polygon
  "Extrude a simple planar polygon along vector [dx dy dz] into a closed
  topological solid. The input loop must have at least three distinct points."
  [points direction]
  (when (or (< (count points) 3) (not= (count points) (count (distinct points))))
    (throw (ex-info "extrusion needs three distinct polygon points" {:points points})))
  (when (every? zero? direction) (throw (ex-info "extrusion direction cannot be zero" {})))
  (let [n (count points) top (mapv #(mapv + % direction) points)
        bottom (vec (reverse (range n))) top-face (vec (range n (* 2 n)))
        sides (mapv (fn [i] (let [j (mod (inc i) n)] [i j (+ n j) (+ n i)])) (range n))
        result (solid (into (vec points) top) (vec (concat [bottom top-face] sides)))]
    (when-not (watertight-solid? result) (throw (ex-info "extrusion produced invalid topology" {})))
    result))

(defn solid-mesh [s]
  (when-not (watertight-solid? s) (throw (ex-info "cannot mesh non-watertight solid" {})))
  (let [triangles (mapcat (fn [face] (map #(vector (first face) (nth face %) (nth face (inc %)))
                                           (range 1 (dec (count face))))) (:solid/faces s))]
    {:positions (:solid/vertices s) :indices (vec (mapcat identity triangles))
     :normals (vec (repeat (count (:solid/vertices s)) [0 0 1]))}))

(defn solid-volume
  "Signed-tetrahedron volume magnitude for a watertight triangulated solid."
  [s]
  (let [{:keys [positions indices]} (solid-mesh s)
        triple (fn [[ax ay az] [bx by bz] [cx cy cz]]
                 (+ (* ax (- (* by cz) (* bz cy)))
                    (* ay (- (* bz cx) (* bx cz)))
                    (* az (- (* bx cy) (* by cx)))))]
    (#?(:clj Math/abs :cljs js/Math.abs)
     (/ (reduce + (map (fn [[a b c]] (triple (nth positions a) (nth positions b) (nth positions c)))
                       (partition 3 indices))) 6.0))))

;; Boolean solid operations: union / difference / intersect.
;;
;; Every solid this engine can currently produce is an `extrude-polygon`
;; result: a planar cross-section polygon rigidly translated by one
;; extrusion vector ("prism"). That gives every solid a product structure
;; (2D cross-section polygon) x (1D interval along the extrusion axis), which
;; is what makes a *genuinely correct*, tractable boolean possible here
;; without a general triangle-mesh/BREP boolean kernel (out of scope --
;; see ADR for the full writeup, following the same narrow-first-cut
;; discipline as org-iso-10303's axis-aligned-cylinder-only revolve).
;;
;; Supported input, for ALL THREE ops (anything else throws ex-info):
;;   - both solids must be watertight `extrude-polygon` prisms (2n vertices,
;;     n+2 faces, top = bottom rigidly translated by one vector);
;;   - the extrusion direction must be axis-aligned (exactly one of x/y/z
;;     nonzero) and the SAME coordinate axis for both solids ("coaxial" in
;;     the sense of a shared axis *direction* -- the two cross-section
;;     polygons may sit anywhere in the plane perpendicular to that axis,
;;     they need not share a common axis *line*);
;;   - each solid's cross-section polygon must be planar (both caps lie
;;     exactly in a plane perpendicular to the axis) and convex.
;;
;; `boolean-intersect` is unconditionally correct for any two such prisms:
;; the Cartesian-product identity (A1xB1) ∩ (A2xB2) = (A1∩A2)x(B1∩B2) holds
;; for arbitrary sets, so (cross-section-polygon intersection) x
;; (axis-interval intersection) IS exactly the solid intersection. Cross-
;; section intersection is computed with Sutherland-Hodgman convex clipping
;; (exact for convex-vs-convex, never introduces a hole).
;;
;; `boolean-union` / `boolean-difference` additionally require the two
;; solids to share an IDENTICAL cross-section polygon (same profile,
;; differing only in the axis interval -- e.g. stacking/cutting the same
;; rounded-rect profile at different heights along its own extrusion axis).
;; This extra restriction is deliberate, not an oversight: the analogous
;; product identity for union/difference does NOT hold in general --
;; (A1xB1) ∪ (A2xB2) != (A1∪A2)x(B1∪B2) unless A1=A2 or B1=B2 -- and general
;; convex-minus-convex cross-section subtraction can produce a
;; multiply-connected result (a polygon WITH A HOLE, e.g. a square frame),
;; which this engine's single-loop `extrude-polygon` cannot represent at
;; all. Rather than ship a booleans that is silently wrong off that
;; boundary, off-profile union/difference (e.g. biting a notch out of the
;; side of a box with a differently-shaped tool) is explicitly NOT
;; supported and throws.
;;
;; NOT supported (throws ex-info in all cases): non-axis-aligned extrusion
;; direction, mismatched extrusion axis between the two solids, non-convex
;; or non-planar cross-sections, self-intersecting polygons, and (for
;; union/difference only) mismatched cross-section profiles.
;;
;; Return shape: all three ops return a *vector* of 0..2 result solids
;; (never a bare solid), so callers have one consistent shape to handle:
;;   - `boolean-intersect` -> `[]` when the solids don't overlap, else `[s]`.
;;   - `boolean-union`     -> `[s]` when the axis intervals touch/overlap
;;     (merged into one solid), else `[a b]` (both originals, unmerged --
;;     this engine does not fuse two disjoint solids into one manifold).
;;   - `boolean-difference` (a minus b) -> `[a]` unchanged when b doesn't
;;     overlap a's interval, `[]` when b fully covers a's interval, else a
;;     vector of 1-2 solids for the remaining slab(s) of a's interval.

(def ^:private axis-index {:x 0 :y 1 :z 2})
(def ^:private plane-indices {:x [1 2] :y [2 0] :z [0 1]})
(def ^:private boolean-eps 1.0e-9)

(defn- direction-axis [direction]
  (let [nonzero (keep-indexed (fn [i v] (when (> (#?(:clj Math/abs :cljs js/Math.abs) v) boolean-eps) i))
                              direction)]
    (when (not= 1 (count nonzero))
      (throw (ex-info "boolean solid ops require an axis-aligned extrusion direction (exactly one of x/y/z nonzero)"
                      {:direction direction})))
    (get [:x :y :z] (first nonzero))))

(defn- polygon-signed-area [points]
  (* 0.5 (reduce + (map (fn [[x1 y1] [x2 y2]] (- (* x1 y2) (* x2 y1)))
                       points (concat (rest points) [(first points)])))))

(defn- ensure-ccw [points]
  (if (neg? (polygon-signed-area points)) (vec (reverse points)) (vec points)))

(defn- convex-polygon?
  "True when `points` (2D, CCW) never turn clockwise at any vertex,
  i.e. the polygon is convex (collinear vertices are tolerated)."
  [points]
  (let [n (count points)
        cross (fn [[ax ay] [bx by] [cx cy]] (- (* (- bx ax) (- cy ay)) (* (- by ay) (- cx ax))))
        signs (for [i (range n)]
                (cross (nth points i) (nth points (mod (inc i) n)) (nth points (mod (+ i 2) n))))]
    (every? #(>= % (- boolean-eps)) signs)))

(defn- dedupe-polygon
  "Drop consecutive (cyclically) near-duplicate points left behind by
  polygon clipping at shared vertices/edges."
  [points]
  (let [close? (fn [a b] (< (reduce + (map (fn [x y] (let [d (- x y)] (* d d))) a b)) 1.0e-18))
        deduped (reduce (fn [acc p] (if (and (seq acc) (close? (peek acc) p)) acc (conj acc p))) [] points)]
    (vec (if (and (> (count deduped) 1) (close? (peek deduped) (first deduped))) (pop deduped) deduped))))

(defn- coaxial-prism
  "Extract `{:axis :ai :i0 :i1 :polygon [[u v]...] :lo :hi}` from a
  watertight solid, or throw if it is not a supported axis-aligned
  convex-polygon `extrude-polygon` prism. `:polygon` is the CCW
  cross-section expressed in the 2D plane perpendicular to `:axis`;
  `:lo`/`:hi` are the axis-coordinate interval the prism spans."
  [s]
  (when-not (watertight-solid? s) (throw (ex-info "boolean solid ops require a watertight solid" {})))
  (let [vertices (:solid/vertices s) total (count vertices) n (quot total 2)]
    (when-not (and (even? total) (>= n 3) (= (count (:solid/faces s)) (+ n 2)))
      (throw (ex-info "boolean solid ops only support extrude-polygon prisms (two n-gon caps + n side quads)"
                      {:vertex-count total :face-count (count (:solid/faces s))})))
    (let [bottom (subvec vertices 0 n) top (subvec vertices n)
          direction (mapv - (first top) (first bottom))]
      (when-not (every? true? (map (fn [b t] (= t (mapv + b direction))) bottom top))
        (throw (ex-info "boolean solid ops only support rigid-translation (prism) solids" {})))
      (let [axis (direction-axis direction) ai (get axis-index axis) [i0 i1] (get plane-indices axis)
            axis-values (mapv #(nth % ai) bottom)]
        (when-not (apply = axis-values)
          (throw (ex-info "boolean solid ops require the cross-section to be planar perpendicular to the extrusion axis"
                          {:axis axis :axis-values axis-values})))
        (let [polygon (ensure-ccw (mapv (fn [v] [(nth v i0) (nth v i1)]) bottom))]
          (when-not (convex-polygon? polygon)
            (throw (ex-info "boolean solid ops only support convex cross-sections" {:polygon polygon})))
          (let [a (first axis-values) b (+ a (nth direction ai))]
            {:axis axis :ai ai :i0 i0 :i1 i1 :polygon polygon :lo (min a b) :hi (max a b)}))))))

(defn- clip-against-edge
  "Sutherland-Hodgman: keep the part of `subject` (any simple polygon) on
  the interior side of the directed edge a->b of a CCW convex clip polygon."
  [subject a b]
  (let [[ax ay] a [bx by] b
        side (fn [[x y]] (- (* (- bx ax) (- y ay)) (* (- by ay) (- x ax))))
        inside? (fn [p] (>= (side p) (- boolean-eps)))
        lerp (fn [c n t] (mapv (fn [cv nv] (+ cv (* t (- nv cv)))) c n))
        edges (map vector subject (concat (rest subject) [(first subject)]))]
    (vec (mapcat (fn [[cur nxt]]
                  (let [cur-in (inside? cur) nxt-in (inside? nxt)]
                    (cond
                      (and cur-in nxt-in) [nxt]
                      (and cur-in (not nxt-in))
                      [(lerp cur nxt (/ (side cur) (- (side cur) (side nxt))))]
                      (and (not cur-in) nxt-in)
                      [(lerp cur nxt (/ (side cur) (- (side cur) (side nxt)))) nxt]
                      :else [])))
                edges))))

(defn- convex-polygon-intersection
  "Exact intersection of two CCW convex polygons via Sutherland-Hodgman
  clipping. Returns `[]` when they don't overlap (or only touch)."
  [subject clip]
  (let [clip-edges (map vector clip (concat (rest clip) [(first clip)]))
        clipped (reduce (fn [poly [a b]] (if (empty? poly) poly (clip-against-edge poly a b))) subject clip-edges)
        deduped (dedupe-polygon clipped)]
    (if (or (< (count deduped) 3) (<= (#?(:clj Math/abs :cljs js/Math.abs) (polygon-signed-area deduped)) boolean-eps))
      []
      deduped)))

(defn- points-close? [a b] (every? true? (map (fn [x y] (< (#?(:clj Math/abs :cljs js/Math.abs) (- x y)) boolean-eps)) a b)))
(defn- rotations [xs] (map #(into (subvec xs %) (subvec xs 0 %)) (range (count xs))))
(defn- same-polygon?
  "Whether two CCW polygons describe the same shape, allowing the vertex
  list to start at a different (but still cyclically-consistent) corner."
  [a b]
  (and (= (count a) (count b))
       (boolean (some (fn [rotated] (every? true? (map points-close? rotated b))) (rotations (vec a))))))

(defn- lift-point [ai i0 i1 axis-value [u v]]
  (-> [0.0 0.0 0.0] (assoc i0 (double u)) (assoc i1 (double v)) (assoc ai (double axis-value))))

(defn- extrude-coaxial-solid
  "Rebuild a solid from prism data `{:axis :ai :i0 :i1}` plus an explicit
  cross-section `polygon` and axis interval `[lo hi]`."
  [{:keys [axis ai i0 i1]} polygon lo hi]
  (let [points (mapv (partial lift-point ai i0 i1 lo) polygon)
        direction (assoc [0.0 0.0 0.0] ai (double (- hi lo)))]
    (extrude-polygon points direction)))

(defn boolean-intersect
  "Solid intersection A ∩ B. See the boolean-ops scope note above this
  section for exactly which solids are supported (axis-aligned coaxial
  convex-polygon prisms; anything else throws). Returns `[]` when the two
  solids don't overlap, else `[result]`."
  [a b]
  (let [pa (coaxial-prism a) pb (coaxial-prism b)]
    (when-not (= (:axis pa) (:axis pb))
      (throw (ex-info "boolean solid ops require both solids to share the same extrusion axis"
                      {:axis-a (:axis pa) :axis-b (:axis pb)})))
    (let [lo (max (:lo pa) (:lo pb)) hi (min (:hi pa) (:hi pb))
          poly (convex-polygon-intersection (:polygon pa) (:polygon pb))]
      (if (or (<= hi lo) (empty? poly))
        []
        [(extrude-coaxial-solid pa poly lo hi)]))))

(defn boolean-union
  "Solid union A ∪ B, scoped to solids sharing an identical cross-section
  profile (see scope note above): the axis intervals are merged when they
  touch or overlap, else both solids are returned unmerged. Throws if the
  two solids don't share the same axis or the same cross-section profile."
  [a b]
  (let [pa (coaxial-prism a) pb (coaxial-prism b)]
    (when-not (= (:axis pa) (:axis pb))
      (throw (ex-info "boolean solid ops require both solids to share the same extrusion axis"
                      {:axis-a (:axis pa) :axis-b (:axis pb)})))
    (when-not (same-polygon? (:polygon pa) (:polygon pb))
      (throw (ex-info "boolean-union only supports solids that share an identical cross-section profile (axis-interval union only)"
                      {})))
    (let [lo (min (:lo pa) (:lo pb)) hi (max (:hi pa) (:hi pb))
          gap? (> (max (:lo pa) (:lo pb)) (min (:hi pa) (:hi pb)))]
      (if gap? [a b] [(extrude-coaxial-solid pa (:polygon pa) lo hi)]))))

(defn boolean-difference
  "Solid difference A − B, scoped to solids sharing an identical
  cross-section profile (see scope note above): subtracts B's axis
  interval from A's, returning the surviving slab(s) of A as 0-2 solids
  with A's original profile. Throws if the two solids don't share the
  same axis or the same cross-section profile."
  [a b]
  (let [pa (coaxial-prism a) pb (coaxial-prism b)]
    (when-not (= (:axis pa) (:axis pb))
      (throw (ex-info "boolean solid ops require both solids to share the same extrusion axis"
                      {:axis-a (:axis pa) :axis-b (:axis pb)})))
    (when-not (same-polygon? (:polygon pa) (:polygon pb))
      (throw (ex-info "boolean-difference only supports solids that share an identical cross-section profile (axis-interval difference only)"
                      {})))
    (let [lo (:lo pa) hi (:hi pa) cut-lo (max lo (:lo pb)) cut-hi (min hi (:hi pb))]
      (if (>= cut-lo cut-hi)
        [a]
        (cond-> []
          (> cut-lo lo) (conj (extrude-coaxial-solid pa (:polygon pa) lo cut-lo))
          (< cut-hi hi) (conj (extrude-coaxial-solid pa (:polygon pa) cut-hi hi)))))))
