(ns kami.cad "Portable rational Bezier/NURBS-compatible CAD primitives.")
(defn curve [points weights] {:cad/kind :nurbs-curve :cad/control-points (vec points) :cad/weights (vec weights)})
(defn- mix [a b t] (mapv #(+ %1 (* (- %2 %1) t)) a b))
(defn evaluate
  "Evaluate a rational Bezier curve with de Casteljau reduction. This is the
  first NURBS-compatible span evaluator; knots/loft compose on top."
  [{:cad/keys [control-points weights]} t]
  (let [h (mapv (fn [[x y z] w] [(* x w) (* y w) (* z w) w]) control-points weights)
        reduce1 (fn [pts] (mapv #(mix %1 %2 t) pts (subvec pts 1)))
        [x y z w] (loop [p h] (if (= 1 (count p)) (first p) (recur (reduce1 p))))]
    [(/ x w) (/ y w) (/ z w)]))
