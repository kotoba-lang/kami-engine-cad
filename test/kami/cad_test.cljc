(ns kami.cad-test (:require [clojure.test :refer [deftest is testing]] [kami.cad :as cad]))

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
