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
