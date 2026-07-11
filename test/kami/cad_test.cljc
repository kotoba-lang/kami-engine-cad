(ns kami.cad-test (:require [clojure.test :refer [deftest is testing]] [kami.cad :as cad]))
(defn close-point? [a b]
  (every? true? (map #(< (#?(:clj Math/abs :cljs js/Math.abs) (- %1 %2)) 1.0e-10) a b)))

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
