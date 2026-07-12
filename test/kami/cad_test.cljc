(ns kami.cad-test (:require [clojure.test :refer [deftest is testing]] [kami.cad :as cad]))
(defn close-point? [a b]
  (every? true? (map #(< (#?(:clj Math/abs :cljs js/Math.abs) (- %1 %2)) 1.0e-10) a b)))
(defn near-point? [a b]
  (every? true? (map #(< (#?(:clj Math/abs :cljs js/Math.abs) (- %1 %2)) 1.0e-6) a b)))

(deftest rational-curve-editing
  (let [c (cad/curve [[0 0 0] [1 2 0] [2 0 0]] [1 1 1])]
    (is (= [1.0 1.0 0.0] (cad/evaluate c 0.5)))
    (is (= 9 (count (cad/tessellate c 8))))
    (is (= [1 3 0] (get-in (cad/move-control-point c 1 [1 3 0]) [:cad/control-points 1])))
    (is (= 2 (get-in (cad/set-weight c 1 2) [:cad/weights 1])))
    (is (= [1.0 2.5 -1.0] (cad/snap-point [1.24 2.49 -1.1] 0.5)))))

(deftest loft-produces-indexed-surface
  (let [a (cad/curve [[-1 0 0] [0 1 0] [1 0 0]] [1 1 1])
        b (cad/curve [[-1 0 2] [0 2 2] [1 0 2]] [1 1 1])
        m (cad/loft-mesh (cad/loft [a b] 8))]
    (is (= 18 (count (:positions m))))
    (is (= 48 (count (:indices m))))
    (is (= (count (:positions m)) (count (:normals m))))))

(deftest exact-split-and-trim
  (let [c (cad/curve [[0 0 0] [1 2 0] [3 1 0] [4 0 0]] [1 2 0.5 1])
        [left right] (cad/split-curve c 0.4)
        trimmed (cad/trim-curve c 0.2 0.8)]
    (is (close-point? (cad/evaluate c 0.4) (cad/evaluate left 1)))
    (is (close-point? (cad/evaluate c 0.4) (cad/evaluate right 0)))
    (is (close-point? (cad/evaluate c 0.2) (cad/evaluate trimmed 0)))
    (is (close-point? (cad/evaluate c 0.8) (cad/evaluate trimmed 1)))
    (is (close-point? (cad/evaluate c 0) (cad/evaluate (cad/reverse-curve c) 1)))))

(deftest tolerance-checked-join
  (let [a (cad/curve [[0 0 0] [1 1 0] [2 0 0]] [1 1 1])
        b (cad/curve [[2 0 0] [3 -1 0] [4 0 0]] [1 1 1])
        joined (cad/join-curves [a b])]
    (is (close-point? [0 0 0] (cad/evaluate-composite joined 0)))
    (is (close-point? [2 0 0] (cad/evaluate-composite joined 0.5)))
    (is (close-point? [4 0 0] (cad/evaluate-composite joined 1)))
    (is (thrown? #?(:clj clojure.lang.ExceptionInfo :cljs js/Error)
                 (cad/join-curves [a (cad/move-control-point b 0 [2.1 0 0])] 0.01)))))

(deftest adaptive-curve-measurement-and-bounds
  (let [w (#?(:clj Math/sqrt :cljs js/Math.sqrt) 0.5)
        quarter-circle (cad/curve [[1 0 0] [1 1 0] [0 1 0]] [1 w 1])
        length (cad/curve-length quarter-circle 1.0e-8)
        box (cad/bounds [[-2 0 1] [3 4 -1] [0 2 5]])]
    (is (< (#?(:clj Math/abs :cljs js/Math.abs) (- length (/ #?(:clj Math/PI :cljs js/Math.PI) 2))) 1.0e-6))
    (is (= {:min [-2 0 -1] :max [3 4 5] :size [5 4 6]} box))
    (is (thrown? #?(:clj clojure.lang.ExceptionInfo :cljs js/Error) (cad/curve-length quarter-circle 0)))
    (is (thrown? #?(:clj clojure.lang.ExceptionInfo :cljs js/Error) (cad/bounds [])))))

(deftest dimensional-sketch-solver
  (let [sketch (cad/sketch [(cad/sketch-point :a 0 0 true) (cad/sketch-point :b 3 0.4)
                            (cad/sketch-point :c 3.2 2)]
                           [(cad/sketch-line :bottom :a :b) (cad/sketch-line :right :b :c)]
                           [(cad/horizontal :h :bottom) (cad/vertical :v :right)
                            (cad/distance-constraint :width :a :b 4)
                            (cad/distance-constraint :height :b :c 2)])
        solved (cad/solve-sketch sketch)]
    (is (true? (get-in solved [:sketch/solver :converged?])))
    (is (< (get-in solved [:sketch/solver :max-residual]) 1.0e-6))
    (is (near-point? [0 0] (get-in solved [:sketch/points :a :sketch.point/position])))
    (is (near-point? [4 0] (get-in solved [:sketch/points :b :sketch.point/position])))
    (is (near-point? [4 2] (get-in solved [:sketch/points :c :sketch.point/position])))))

(deftest coincident-and-overconstrained-diagnostics
  (let [base (cad/sketch [(cad/sketch-point :a 0 0 true) (cad/sketch-point :b 1 1)] []
                         [(cad/coincident :same :a :b)])
        solved (cad/solve-sketch base)
        impossible (cad/solve-sketch (update base :sketch/points assoc-in [:b :sketch.point/fixed?] true))]
    (is (close-point? [0 0] (get-in solved [:sketch/points :b :sketch.point/position])))
    (is (true? (get-in solved [:sketch/solver :converged?])))
    (is (false? (get-in impossible [:sketch/solver :converged?])))
    (is (pos? (get-in impossible [:sketch/solver :max-residual])))))

(deftest deterministic-parametric-feature-history
  (let [base (cad/curve [[0 0 0] [1 2 0] [2 0 0]] [1 1 1])
        model (cad/feature-model [(cad/feature :sketch :source [] {:value base})
                                  (cad/feature :move :move-control-point [:sketch] {:index 1 :point [1 4 0]})
                                  (cad/feature :trim :trim [:move] {:t0 0.25 :t1 0.75})])
        rebuilt (cad/recompute-feature-model model)
        edited (-> model (cad/update-feature :move assoc-in [:feature/params :point] [1 6 0])
                   cad/recompute-feature-model)]
    (is (= {:sketch {:status :ok} :move {:status :ok} :trim {:status :ok}}
           (:feature-model/statuses rebuilt)))
    (is (not= (cad/evaluate (get-in rebuilt [:feature-model/results :trim]) 0.5)
              (cad/evaluate (get-in edited [:feature-model/results :trim]) 0.5)))
    (is (= :suppressed (get-in (cad/recompute-feature-model (cad/suppress-feature model :move true))
                               [:feature-model/statuses :move :status])))
    (is (= :blocked (get-in (cad/recompute-feature-model (cad/suppress-feature model :move true))
                            [:feature-model/statuses :trim :status])))))

(deftest feature-history-diagnostics-preserve-independent-branches
  (let [c (cad/curve [[0 0 0] [1 1 0]] [1 1])
        model (cad/feature-model [(cad/feature :bad :trim [:later] {:t0 0 :t1 1})
                                  (cad/feature :independent :source [] {:value c})
                                  (cad/feature :later :source [] {:value c})])
        rebuilt (cad/recompute-feature-model model)]
    (is (= :error (get-in rebuilt [:feature-model/statuses :bad :status])))
    (is (= :input-not-before-feature (get-in rebuilt [:feature-model/statuses :bad :reason])))
    (is (= :ok (get-in rebuilt [:feature-model/statuses :independent :status])))
    (is (thrown? #?(:clj Exception :cljs js/Error)
                 (cad/feature-model [(cad/feature :same :source [] {:value c})
                                     (cad/feature :same :source [] {:value c})])))))

(deftest watertight-polygon-extrusion
  (let [box (cad/extrude-polygon [[0 0 0] [4 0 0] [4 3 0] [0 3 0]] [0 0 2])
        mesh (cad/solid-mesh box)]
    (is (cad/watertight-solid? box))
    (is (= 8 (count (:solid/vertices box))))
    (is (= 6 (count (:solid/faces box))))
    (is (= 12 (count (cad/solid-edges box))))
    (is (= 36 (count (:indices mesh))))
    (is (== 24.0 (cad/solid-volume box)))))

(deftest solid-topology-validation
  (let [open (cad/solid [[0 0 0] [1 0 0] [0 1 0]] [[0 1 2]])]
    (is (false? (cad/watertight-solid? open)))
    (is (thrown? #?(:clj Exception :cljs js/Error) (cad/solid-mesh open)))
    (is (thrown? #?(:clj Exception :cljs js/Error)
                 (cad/extrude-polygon [[0 0 0] [1 0 0] [1 0 0]] [0 0 1])))
    (is (thrown? #?(:clj Exception :cljs js/Error)
                 (cad/extrude-polygon [[0 0 0] [1 0 0] [0 1 0]] [0 0 0])))))

(defn- square-sketch []
  (cad/sketch [(cad/sketch-point :p0 0 0 true) (cad/sketch-point :p1 4 0 true)
               (cad/sketch-point :p2 4 4 true) (cad/sketch-point :p3 0 4 true)]
              [(cad/sketch-line :l0 :p0 :p1) (cad/sketch-line :l1 :p1 :p2)
               (cad/sketch-line :l2 :p2 :p3) (cad/sketch-line :l3 :p3 :p0)]
              []))

(deftest fillet-sketch-rounds-a-right-angle-corner
  (let [filleted (cad/fillet-sketch (square-sketch) :p1 1 :fs :fe :farc)
        start (get-in filleted [:sketch/points :fs :sketch.point/position])
        end (get-in filleted [:sketch/points :fe :sketch.point/position])
        arc (first (filter #(= :farc (:sketch.entity/id %)) (:sketch/entities filleted)))
        center (:sketch.entity/center arc)
        l0 (first (filter #(= :l0 (:sketch.entity/id %)) (:sketch/entities filleted)))
        l1 (first (filter #(= :l1 (:sketch.entity/id %)) (:sketch/entities filleted)))]
    (is (near-point? [3.0 0.0] start))
    (is (near-point? [4.0 1.0] end))
    (is (near-point? [3.0 1.0] center))
    (is (= :fs (:sketch.entity/b l0)))
    (is (= :fe (:sketch.entity/a l1)))
    ;; tangency: center->start is perpendicular to l0's direction, center->end to l1's
    (is (< (#?(:clj Math/abs :cljs js/Math.abs)
            (reduce + (map * (map - start center) [1 0]))) 1.0e-9))
    (is (< (#?(:clj Math/abs :cljs js/Math.abs)
            (reduce + (map * (map - end center) [0 1]))) 1.0e-9))
    (is (thrown? #?(:clj clojure.lang.ExceptionInfo :cljs js/Error)
                 (cad/fillet-sketch (square-sketch) :p1 5 :fs :fe :farc)))
    (is (thrown? #?(:clj clojure.lang.ExceptionInfo :cljs js/Error)
                 (cad/fillet-sketch (square-sketch) :p1 -1 :fs :fe :farc)))))

(deftest chamfer-sketch-cuts-a-right-angle-corner
  (let [chamfered (cad/chamfer-sketch (square-sketch) :p1 1 :cs :ce :cseg)
        start (get-in chamfered [:sketch/points :cs :sketch.point/position])
        end (get-in chamfered [:sketch/points :ce :sketch.point/position])
        seg (first (filter #(= :cseg (:sketch.entity/id %)) (:sketch/entities chamfered)))]
    (is (near-point? [3.0 0.0] start))
    (is (near-point? [4.0 1.0] end))
    (is (= :cs (:sketch.entity/a seg)))
    (is (= :ce (:sketch.entity/b seg)))
    (is (thrown? #?(:clj clojure.lang.ExceptionInfo :cljs js/Error)
                 (cad/chamfer-sketch (square-sketch) :p1 4 :cs :ce :cseg)))))

(deftest filleted-sketch-loop-extrudes-watertight
  (let [filleted (cad/fillet-sketch (square-sketch) :p1 1 :fs :fe :farc)
        polygon (cad/sketch-loop->polygon filleted 8)
        solid (cad/extrude-polygon polygon [0 0 2])]
    (is (= 12 (count polygon)))
    (is (= (count polygon) (count (distinct polygon))))
    (is (cad/watertight-solid? solid))
    (is (= (* 2 (count polygon)) (count (:solid/vertices solid))))))

(deftest chamfered-sketch-loop-extrudes-watertight
  (let [chamfered (cad/chamfer-sketch (square-sketch) :p1 1 :cs :ce :cseg)
        polygon (cad/sketch-loop->polygon chamfered)
        solid (cad/extrude-polygon polygon [0 0 2])]
    (is (= 5 (count polygon)))
    (is (cad/watertight-solid? solid))))

(deftest fillet-sketch-participates-in-feature-tree
  (let [model (cad/feature-model
               [(cad/feature :sketch :source [] {:value (square-sketch)})
                (cad/feature :round :fillet-sketch [:sketch]
                            {:corner :p1 :radius 1 :start-id :fs :end-id :fe :arc-id :farc})
                (cad/feature :profile :sketch->polygon [:round] {:segments 8})
                (cad/feature :solid :extrude [:profile] {:direction [0 0 2]})])
        rebuilt (cad/recompute-feature-model model)]
    (is (= :ok (get-in rebuilt [:feature-model/statuses :solid :status])))
    (is (cad/watertight-solid? (get-in rebuilt [:feature-model/results :solid])))))
